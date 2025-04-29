#include "ConfigParserHelpers.h"

#include <iostream>

namespace SaiInterfaces {
namespace ConfigParserHelpers {
using namespace std;
using namespace Eigen;

Quaterniond rpyToQuaternion(double roll, double pitch, double yaw) {
	AngleAxisd rollAA(roll, Vector3d::UnitX());
	AngleAxisd pitchAA(pitch, Vector3d::UnitY());
	AngleAxisd yawAA(yaw, Vector3d::UnitZ());

	Quaterniond q = yawAA * pitchAA * rollAA;
	return q;
}

vector<string> splitString(const string& str, const vector<char>& separators) {
	vector<string> tokens;
	istringstream iss(str);
	string token;

	while (getline(iss, token)) {
		string current;
		bool inToken = false;

		for (char ch : token) {
			if (find(separators.begin(), separators.end(), ch) !=
				separators.end()) {
				if (inToken) {
					tokens.push_back(current);
					current.clear();
					inToken = false;
				}
			} else {
				current += ch;
				inToken = true;
			}
		}

		if (inToken) {
			tokens.push_back(current);
		}
	}

	return tokens;
}

Vector3d parseVector3d(const char* vec_str) {
	vector<string> vectorStr = splitString(vec_str);
	if (vectorStr.size() != 3) {
		throw runtime_error(
			"xml parsing error. Cannot parse Vector3d from the string " +
			string(vec_str) + " because it should have 3 values.");
	}

	Vector3d vec;
	try {
		vec = Vector3d(stod(vectorStr[0]), stod(vectorStr[1]),
					   stod(vectorStr[2]));
	} catch (exception& e) {
		cout << e.what() << endl;
		throw runtime_error("Could not parse Vector3d from the string " +
							string(vec_str) +
							" because one of the values is not a number.");
	}

	return vec;
}

Vector3d parseVector3d(tinyxml2::XMLElement* xml, string attribute_name) {
	try {
		return parseVector3d(xml->Attribute(attribute_name.c_str()));
	} catch (exception& e) {
		cout << e.what() << endl;
		throw runtime_error("Could not parse Vector3d from xml element " +
							string(xml->Name()) + " with attribute " +
							attribute_name);
	}
}

VectorXd parseVectorXd(const char* vec_str) {
	vector<string> vectorStr = splitString(vec_str);
	const int size = vectorStr.size();
	VectorXd vec = VectorXd(size);

	try {
		for (int i = 0; i < size; i++) {
			vec[i] = stod(vectorStr[i]);
		}
	} catch (const std::exception& e) {
		std::cerr << e.what() << '\n';
		throw runtime_error("Could not parse VectorXd from the string " +
							string(vec_str) +
							" because one of the values is not a number.");
	}

	return vec;
}

Affine3d parsePose(tinyxml2::XMLElement* xml) {
	Affine3d pose = Affine3d::Identity();

	try {
		Vector3d position = parseVector3d(xml, "xyz");
		Vector3d rpy_vector = parseVector3d(xml, "rpy");

		pose.translation() = position;
		pose.linear() =
			rpyToQuaternion(rpy_vector[0], rpy_vector[1], rpy_vector[2])
				.toRotationMatrix();
	} catch (exception& e) {
		cout << e.what() << endl;
		throw runtime_error("Could not parse pose from xml element " +
							string(xml->Name()));
	}
	return pose;
}

LoggerConfig parseLoggerConfig(tinyxml2::XMLElement* logger,
							   const std::string& default_folder_name) {
	LoggerConfig config = LoggerConfig(default_folder_name);

	if (logger->Attribute("logFolderName")) {
		config.folder_name = logger->Attribute("logFolderName");
	}
	if (logger->Attribute("logFrequency")) {
		config.frequency = logger->DoubleAttribute("logFrequency");
	}
	if (logger->Attribute("enabledAtStartup")) {
		config.start_with_logger_on = logger->BoolAttribute("enabledAtStartup");
	}
	if (logger->Attribute("addTimestampToFilename")) {
		config.add_timestamp_to_filename =
			logger->BoolAttribute("addTimestampToFilename");
	}

	return config;
}

RedisConfig parseRedisConfig(tinyxml2::XMLElement* redis) {
	RedisConfig config;

	if (redis->Attribute("ip")) {
		config.redis_ip = redis->Attribute("ip");
	}
	if (redis->Attribute("port")) {
		config.redis_port = redis->IntAttribute("port");
	}
	if (redis->Attribute("namespacePrefix")) {
		config.redis_namespace_prefix = redis->Attribute("namespacePrefix");
	}

	return config;
}

}  // namespace ConfigParserHelpers
}  // namespace SaiInterfaces