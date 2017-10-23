#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("]");
	if (found_null != std::string::npos) {
		return "";
	} else if (b1 != std::string::npos && b2 != std::string::npos) {
		return s.substr(b1, b2 - b1 + 1);
	}
	return "";
}

int main() {
	uWS::Hub h;

	// Create a UKF instance
	UKF ukf;
	long long previous_timestamp;
	double dt;						// save dt because often measurements are coming it at the same time

#ifdef USE_LATEST_UWS
#else
#endif

#ifdef USE_LATEST_UWS
	h.onMessage([&ukf, &previous_timestamp, &dt](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
#else
	h.onMessage([&ukf, &previous_timestamp, &dt](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
#endif
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event

		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(std::string(data));
			if (s != "") {


				auto j = json::parse(s);
				std::string event = j[0].get<std::string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					double hunter_x = std::stod(j[1]["hunter_x"].get<std::string>());
					double hunter_y = std::stod(j[1]["hunter_y"].get<std::string>());
					double hunter_heading = std::stod(j[1]["hunter_heading"].get<std::string>());

					string lidar_measurment = j[1]["lidar_measurement"];

					MeasurementPackage meas_package_L;
					istringstream iss_L(lidar_measurment);
					long long timestamp_L;

					// reads first element from the current line
					string sensor_type_L;
					iss_L >> sensor_type_L;

					meas_package_L.sensor_type_ = MeasurementPackage::LASER;
					meas_package_L.raw_measurements_ = VectorXd(2);
					float px;
					float py;
					iss_L >> px;
					iss_L >> py;
					meas_package_L.raw_measurements_ << px, py;
					iss_L >> timestamp_L;
					meas_package_L.timestamp_ = timestamp_L;

					ukf.ProcessMeasurement(meas_package_L);

					string radar_measurment = j[1]["radar_measurement"];

					MeasurementPackage meas_package_R;
					istringstream iss_R(radar_measurment);
					long long timestamp_R;

					// reads first element from the current line
					string sensor_type_R;
					iss_R >> sensor_type_R;

					// read measurements at this timestamp
					meas_package_R.sensor_type_ = MeasurementPackage::RADAR;
					meas_package_R.raw_measurements_ = VectorXd(3);
					float ro;
					float theta;
					float ro_dot;
					iss_R >> ro;
					iss_R >> theta;
					iss_R >> ro_dot;
					meas_package_R.raw_measurements_ << ro, theta, ro_dot;
					iss_R >> timestamp_R;
					meas_package_R.timestamp_ = timestamp_R;

					ukf.ProcessMeasurement(meas_package_R);

					double target_x = ukf.x_[0];
					double target_y = ukf.x_[1];
					double target_v = ukf.x_[2];
					double target_yaw = ukf.x_[3];
					double target_yaw_d = ukf.x_[4];

					double dt_new = (min(timestamp_L, timestamp_R) - previous_timestamp) / 1000000.0;
					if (dt_new > 0.00001) {
						dt = dt_new;
					}

					double yaw_to_target = 0;
					double yaw_difference = 0;
					double distance = 0;
					double hunter_yaw = 0;
					double hunter_v = 0;

					if (ukf.P_(4, 4) < 0.002) {           // wait for low covariance
						double pred_x = target_x;		  // target predicted px
						double pred_y = target_y;         // target predicted py
						double pred_yaw = target_yaw;     // target predicted yaw

						double steps = 0;

						// step forward in dt increments and check when target can be intercepted
						// while hunter moving at maximum of target's speed
						do {
							steps += 1;
							pred_yaw = Tools::ConstrainRadian(pred_yaw + dt * target_yaw_d);
							pred_x += dt * target_v * cos(pred_yaw);
							pred_y += dt * target_v * sin(pred_yaw);
							yaw_to_target = Tools::ConstrainRadian(atan2(pred_y - hunter_y, pred_x - hunter_x));
							yaw_difference = Tools::ConstrainRadian(yaw_to_target - hunter_heading);
							distance = sqrt((pred_y - hunter_y) * (pred_y - hunter_y) + (pred_x - hunter_x) * (pred_x - hunter_x));
							hunter_v = distance / (dt * steps);
						} while (hunter_v > target_v);
						hunter_yaw = yaw_difference;
					}

					json msgJson;
					msgJson["turn"] = hunter_yaw;
					msgJson["dist"] = hunter_v;
					auto msg = "42[\"move_hunter\"," + msgJson.dump() + "]";
					//std::cout << msg << std::endl;
#ifdef USE_LATEST_UWS
					ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
					previous_timestamp = min(timestamp_L, timestamp_R);

				}
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
#ifdef USE_LATEST_UWS
				ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
			}
		}

	});

	// We don't need this since we're not using HTTP but if it's removed the program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse * res, uWS::HttpRequest req, char *data, size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

#ifdef USE_LATEST_UWS
	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
#else
	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
#endif
		std::cout << "Connected!!!" << std::endl;
	});

#ifdef USE_LATEST_UWS
	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
		ws->close();
#else
	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
		ws.close();
#endif
		std::cout << "Disconnected" << std::endl;
	});

	auto host = "127.0.0.1";
	int port = 4567;
	if (h.listen(host, port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}

