#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <tuple>
#include <iomanip>
#include <string>
#include <cmath>
#include <krpc.hpp>
#include <krpc/services/space_center.hpp>
//Some useful constants
#define g 9.807 			//Standard Earth ASL gravitational acceleration
#define h 0.01  	 		//Second-order algorithm timestep magnitude
#define PI 3.14159265359	//Pi (duh) 

class Stage {
public:
	double start_TWR;
	double end_TWR;
	double burn_time;
	bool ullage;
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

void check_stages(krpc::services::SpaceCenter::Control cont, 
				  std::vector<krpc::services::SpaceCenter::Engine> engines, 
				  krpc::services::SpaceCenter::Engine& active_eng,
				  std::vector<Stage>::iterator& current_stage) {
	if(active_eng.thrust() == 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			cont.activate_next_stage();
			if(current_stage->ullage) {
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
				cont.activate_next_stage();
			}
			for(auto engine : engines) //Find the next stage's engine
				if(engine.active() && engine.has_fuel()) {
					active_eng = engine;
					break;
				}
			current_stage++;
		}
}

int main(int argc, char* argv[]) {
    krpc::Client conn = krpc::connect();
	krpc::services::SpaceCenter space_center(&conn);
	auto vessel = space_center.active_vessel();
	std::vector<double> v; 					//Proper storage of velocity values
	std::vector<double> beta;				//Proper storage of beta (angle off of vertical)
	std::vector<double> v_temp = {0};		//Temporary v storage for use in second-order approximation
	std::vector<double> beta_temp = {0};	//Ditto for beta
	std::vector<Stage> stages;
	std::string filepath = argv[1];
	filepath += ".txt";
	std::ifstream istr(filepath);
	
	istr >> beta_temp[0];
	
	double temp;
	int counter = 0;
	//Use a switch to parse the file (the idea is to insert error-checking for an incomplete file later)
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
				counter++;
				break;
			case 3:
				stages.back().ullage = temp;
				counter = 0;
		}
	}
	
	std::cout << "Computing gravity turn...\n";
	
	//TODO: Fix all of this mess so it ain't borked
	double t = 0, acc_t = 0;
	int i = 0;
	//Actually account for staging properly (flarping non-smoothness of TWR...)
	for(std::vector<Stage>::iterator stage = stages.begin(); stage != stages.end(); stage++) {
		//Temporaryer vectors used for first-order approximation
		std::vector<double> v_fo; v_fo.push_back(v_temp.back());
		std::vector<double> beta_fo; beta_fo.push_back(beta_temp.back());
		acc_t += stage->burn_time;
		//Use first-order algorithm at 100x precision to get first 4 terms
		for(int j = 0; j < 30; j++) {
			v_fo.push_back(.001*g*(get_TWR(stages, t + .001) - cos(beta_fo[j])) + v_fo[j]);
			double v_beta = .001*g*sin(beta_fo[j]) + beta_fo[j]*v_fo[j+1];
			beta_fo.push_back(v_beta / v_fo[j+1]);
			t += .001;
		}
		
		//Believe it or not, but these are not the silliest for loops I've ever written.
		//That one that "incremented" by multiplying the index by 10 was definitely sillier
		
		int inc = i + 4;
		//Move first 4 terms into place (don't need to move the first since it's already in place)
		for(i++; i < inc; i++) {
			v_temp.push_back(v_fo[(4 - ((inc - i) % 4)) * 10]); //Ain't that a fun bit of math...
			beta_temp.push_back(beta_fo[(4 - ((inc - i) % 4)) * 10]);
		}
		
		//Compute gravity turn
		for(i--; i < acc_t / h; i++) {
			double b = beta_temp[i-1] + (beta_temp[i] - beta_temp[i-1])*2;
			//Yay, magic coefficients
			v_temp.push_back((12.0/25.0) * h * g * (get_TWR(stages, t + h) - cos(b)) + 
							 (48.0/25.0) * v_temp[i] - (36.0/25.0) * v_temp[i-1] + 
							 (16.0/25.0) * v_temp[i-2] - (3.0/25.0) * v_temp[i-3]);
			beta_temp.push_back((12.0/25.0) * ((h * g) / v_temp[i+1]) * sin(b) + 
								(48.0/25.0) * beta_temp[i] - (36.0/25.0) * beta_temp[i-1] + 
								(16.0/25.0) * beta_temp[i-2] - (3.0/25.0) * beta_temp[i-3]);
			t += h;
		}
		//Implement this later
		if(stage != --stages.end()) { //Woo, prefix-decrement
			
		}
	}
	
	for(unsigned j = 0; j < v_temp.size() / 10; j++){
		v.push_back(v_temp[j*10]);
		beta.push_back(beta_temp[j*10]);
	}
	
	std::cout << "Done.\n\n";
	
	
	auto ut = space_center.ut_stream();
	//Prep for launch
	vessel.control().set_sas(false);
	vessel.control().set_rcs(true);
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
	auto apoapsis = vessel.orbit().apoapsis_altitude_stream();
	auto gees = vessel.flight(vessel.orbit().body().reference_frame()).g_force_stream();
	krpc::services::SpaceCenter::Engine active_eng;
	auto engines = vessel.parts().engines();
	
	ap.set_reference_frame(ref_frame);
	ap.set_target_direction(std::make_tuple(1, 0, 0));
	
	std::this_thread::sleep_for(std::chrono::seconds(1));
	std::cout << "Main sequence start\n";
	cont.activate_next_stage();
	for(auto engine : engines)
		if(engine.active() && engine.has_fuel())
			active_eng = engine;
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
	
	bool fairing_sep = false;
	double tgt_pitch = 0;
	
	auto stage_itr = stages.begin();
	
	ap.set_target_roll(0);
	
	std::cout << "Executing gravity turn...\n";
	//Actually execute gravity turn
	for(unsigned j = 1; j < v.size(); j++) {
		if(v[j] > v[j-1])
			while(velocity() < v[j]) check_stages(cont, engines, active_eng, stage_itr);
		else
			while(velocity() > v[j]) check_stages(cont, engines, active_eng, stage_itr);
		std::tuple<double, double, double> direction = std::make_tuple(cos(beta[j]), tgt_pitch, sin(beta[j]));
		ap.set_target_direction(direction);
	//	std::cout << std::get<0>(direction) << " " << std::get<1>(direction) << " " << std::get<2>(direction) << "\n";
		
		if(altitude() > 100000){
			if(!fairing_sep){
				cont.activate_next_stage();
				fairing_sep = true;
			}
			tgt_pitch = std::get<1>(normalize(vessel.flight(velocity_frame).velocity()));
		}
		if(abs(apoapsis() - periapsis()) < 10000) {
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