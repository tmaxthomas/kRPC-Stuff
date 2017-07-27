#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <tuple>
#include <iomanip>
#include <cmath>
#include <krpc.hpp>
#include <krpc/services/space_center.hpp>
//Some useful constants
#define g 9.807 			//Standard Earth ASL gravitational acceleration
#define h 0.1  	 			//Second-order algorithm timestep magnitude
#define PI 3.14159265359	//Pi (duh) 

class Stage {
public:
	double start_TWR;
	double end_TWR;
	double burn_time;
};

double get_TWR(std::vector<Stage> stages, double time) {
	int stage_num = -1;
	double acc_time = 0; //Holds useful time numbers
	while(acc_time <= time) {
		stage_num++;
		acc_time += stages[stage_num].burn_time;
	}
	acc_time -= stages[stage_num].burn_time;
	acc_time = time - acc_time;
	Stage stage = stages[stage_num];
	double m_end = 1.0 / stage.end_TWR; //Fake-unit start/end mass numbers
	double m_start = 1.0 / stage.start_TWR;
	double m_inc = (m_start - m_end) / stage.burn_time;
	return 1.0 / (m_start - (m_inc * acc_time));
}

//Vector normalizer
std::tuple<double, double, double> normalize(std::tuple<double, double, double> vec) {
	double mag = sqrt(pow(std::get<0>(vec), 2) + pow(std::get<1>(vec), 2) + pow(std::get<2>(vec), 2));
	return std::make_tuple(std::get<0>(vec)/mag, std::get<1>(vec)/mag, std::get<2>(vec)/mag);
}

int main() {
    krpc::Client conn = krpc::connect();
	krpc::services::SpaceCenter space_center(&conn);
	auto vessel = space_center.active_vessel();
	std::vector<double> v = {0};
	std::vector<double> beta = {0.00000000415};  //This number is magic. Don't change it.
	std::vector<Stage> stages;
	
	std::ifstream istr("config.txt");
	double temp;
	int counter = 0;
	while(istr >> temp) {
		switch(counter) {
			case 0:
				Stage new_stage;
				new_stage.start_TWR = temp;
				stages.push_back(new_stage);
				counter++;
				break;
			case 1:
				stages.back().end_TWR = temp;
				counter++;
				break;
			case 2:
				stages.back().burn_time = temp;
				counter = 0;
		}
	}
	
	std::cout << "Computing gravity turn...\n";
	
	//Use first-order algorithm at 100x precision to get first 4 terms
	for(double i = 0; i < 300; i++) {
		v.push_back(.001*g*(get_TWR(stages, .001*(i+1)) - cos(beta[i])) + v[i]);
		double v_beta = .001*g*sin(beta[i]) + beta[i]*v[i+1];
		beta.push_back(v_beta / v[i+1]);
	}
	
	//Move first 4 terms into place & truncate vector
	for(int i = 1; i < 4; i++) {
		v[i] = v[i*100];
		beta[i] = beta[i*100];
	}
	
	v.resize(4);
	beta.resize(4);
	//Compute 500 seconds of gravity turn
	for(double i = 3; i < 5000; i++) {
		double t = i * h;
		double b = beta[i-1] + (beta[i] - beta[i-1])*2;
		//Yay, magic coefficients
		v.push_back((12.0/25.0) * h * g * (get_TWR(stages, t + h) - cos(b)) + (48.0/25.0) * v[i] - (36.0/25.0) * v[i-1] + (16.0/25.0) * v[i-2] - (3.0/25.0) * v[i-3]);
		beta.push_back((12.0/25.0) * ((h * g) / v[i+1]) * sin(b) + (48.0/25.0) * beta[i] - (36.0/25.0) * beta[i-1] + (16.0/25.0) * beta[i-2] - (3.0/25.0) * beta[i-3]);
	}
	
	std::cout << "Done.\n\n";
	
	
	auto ut = space_center.ut_stream();
	//Prep for launch
	vessel.control().set_sas(false);
	vessel.control().set_rcs(false);
	vessel.control().set_throttle(1);
	
	auto ap = vessel.auto_pilot();
	auto cont = vessel.control();
	ap.engage();
	
	auto ref_frame = vessel.surface_reference_frame();
	auto velocity_frame = krpc::services::SpaceCenter::ReferenceFrame::create_hybrid(conn, 
		vessel.orbit().body().non_rotating_reference_frame(),
		vessel.surface_reference_frame());
	auto velocity = vessel.flight(vessel.orbit().body().reference_frame()).speed_stream();
	auto altitude = vessel.flight().mean_altitude_stream();
	auto periapsis = vessel.orbit().periapsis_altitude_stream();
	auto gees = vessel.flight(vessel.orbit().body().reference_frame()).g_force_stream();
	krpc::services::SpaceCenter::Engine booster_eng;
	auto engines = vessel.parts().engines();
	for(auto engine : engines) {
		if(engine.part().title() == "LR89 Series") {
			booster_eng = engine;
			break;
		}
	}
	
	ap.set_reference_frame(ref_frame);
	ap.set_target_direction(std::make_tuple(1, 0, 0));
	
	std::this_thread::sleep_for(std::chrono::seconds(1));
	std::cout << "Main sequence start\n";
	cont.activate_next_stage();
	std::cout << "3...\n";
	std::this_thread::sleep_for(std::chrono::seconds(1));
	std::cout << "2...\n";
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	std::cout << "All engines running\n";
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	std::cout << "1...\n";
	std::this_thread::sleep_for(std::chrono::seconds(1));
	cont.activate_next_stage();
	std::cout << "Liftoff!\n\n";
	
	bool booster_sep = false, fairing_sep = false;
	double tgt_pitch = 0;
	
	std::cout << "Executing gravity turn...\n";
	//Actually execute gravity turn
	for(unsigned i = 1; i < v.size(); i++) {
		while(velocity() < v[i]);
		std::tuple<double, double, double> direction = std::make_tuple(cos(beta[i]), tgt_pitch, sin(beta[i]));
		ap.set_target_direction(direction);
		if(!booster_sep && booster_eng.thrust() == 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			cont.activate_next_stage();
			cont.set_rcs(true);
			booster_sep = true;
		}
		if(altitude() > 100000){
			if(!fairing_sep){
				cont.activate_next_stage();
			fairing_sep = true;
			}
			tgt_pitch = std::get<1>(normalize(vessel.flight(velocity_frame).velocity()));
		}
		if(periapsis() > 170000) {
			std::cout << "Done.\n\n";
			cont.set_throttle(0);
			std::this_thread::sleep_for(std::chrono::seconds(1));
			cont.activate_next_stage();
			break;
		}
	}
	
	std::cout << "Orbit achieved";
    return 0;
}