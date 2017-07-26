#include <krpc.hpp>
#include <krpc/services/space_center.hpp>

#include <fstream>

int main(int argc, char* argv[]) {
	krpc::Client conn = krpc::connect();
	krpc::services::SpaceCenter sc(&conn);
	auto vessel = sc.active_vessel();
	auto v = sc.
}