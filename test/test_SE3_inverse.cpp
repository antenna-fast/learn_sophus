#include <iostream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

#include <Eigen/Core>
#include <sophus/so3.h>
#include <sophus/se3.h>

#include <opencv2/opencv.hpp>


using namespace std;
namespace fs = boost::filesystem;


int get_filenames(const std::string& dir, std::vector<std::string>& filenames)
{
	fs::path path(dir);
	if (!fs::exists(path))
	{
		return -1;
	}
 
	fs::directory_iterator end_iter;
	for (fs::directory_iterator iter(path); iter!=end_iter; ++iter)
	{
		if (fs::is_regular_file(iter->status()))
		{
			filenames.push_back(iter->path().string());
		}
 
		if (fs::is_directory(iter->status()))
		{
			get_filenames(iter->path().string(), filenames);
		}
	}
 
	return filenames.size();
}

bool Tcw2Twc(Eigen::Vector3d trans, Eigen::Quaterniond q, Sophus::SE3 &T_output)
{
  // world to camera
  Sophus::SE3 T_input(q, trans);
  // cout << "T_input: " << T_input.translation()[0] << ", " << T_input.translation()[1] << ", " << T_input.translation()[2] << endl;
  cout << "T_input: \n" << T_input << endl;

  // pose: camera to world
  vector<float> pose(7);
  T_output = T_input.inverse();
  pose[0] = T_output.unit_quaternion().w();
  pose[1] = T_output.unit_quaternion().x();
  pose[2] = T_output.unit_quaternion().y();
  pose[3] = T_output.unit_quaternion().z();
  pose[4] = T_output.translation()[0];
  pose[5] = T_output.translation()[1];
  pose[6] = T_output.translation()[2];
  cout << "T_output: \n" << T_output << endl;

  return true;
}


int main(int argc, char **argv) {

    string data_root = "/home/antenna/ssd/";
    string phone_type = "vivo_iqoo_neo";

    string yml_path = data_root + "/" + phone_type + "/images";

    // write to file
    ofstream output_file;      
    output_file.open("test.txt");  // open file 

    vector<string> file_list;
    int res = get_filenames(yml_path, file_list);
    if(res == -1) 
    {
        cout << "Failed to read file list." << endl;
    }

    sort(yml_path.begin(), yml_path.end());

    // string target = "1673439937797.jpg.yml";
    string target = "jpg.yml";
    string::size_type position;
    string substr;

    for (auto &fname : file_list) 
    {
      string::size_type idx;
      idx = fname.find(target);
      if(idx == string::npos)
      {
        // cout << "target " << target << " not found!" << endl;
        continue;;
      }
      else
      {
        cout << "Found fname: " << fname << endl;
        // load yml
        cv::FileStorage fs;
        fs.open(fname, cv::FileStorage::READ);
        cv::Mat point3d, point2d, R, t;
        // fs["point3d"] >> point3d;
        // fs["point2d"] >> point2d;
        fs["R"] >> R;
        fs["t"] >> t;
        cout << "t: " << t << endl;
        fs.release();

        // get Twc
        Eigen::Vector3d trans(t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0));
        Eigen::Quaterniond q(R.at<double>(0,0), R.at<double>(1,0), R.at<double>(2,0), R.at<double>(3,0));
        Sophus::SE3 SE3_qt(q, trans);
        // cout << "t in SE3: " << SE3_qt.translation()[0] << ", " << SE3_qt.translation()[1] << ", " << SE3_qt.translation()[2] << endl;

        Sophus::SE3 SE3_qt_inverse = SE3_qt.inverse();
        cout << "SE3_qt_inverse: \n" << SE3_qt_inverse.translation()[0] << ", " << SE3_qt_inverse.translation()[1] << ", " << SE3_qt_inverse.translation()[2] << endl;
        
        // save t to file
        // write to file stream
        position = fname.find_first_of(".");
        substr = fname.substr(0, position);
      
        position = substr.find_last_of("/");
        substr = substr.substr(position+1);
                
        output_file << substr << ", " << SE3_qt_inverse.translation()[0] << ", " << SE3_qt_inverse.translation()[1] << ", " << SE3_qt_inverse.translation()[2] << "\n";
      }
    }

    // close file
    output_file.close();

    return 0;
}