#include <opencv2/opencv.hpp>

#ifndef KALMAN_H_INCLUDE
#define KALMAN_H_INCLUDE

using namespace cv;
using namespace std;


namespace kalmantracking {
	
	template<typename T>
	class Max_Size_Vector {
	private:
		unsigned long _max_size;
		std::vector<T> _vec;
	public:
		Max_Size_Vector();
		Max_Size_Vector(unsigned long max_size);
		
		void push_back(const T _var);
		std::vector<T> getVector();
		unsigned long size();
		T& operator[](int i);
	
	};
	
	class kalman {
	private:
		cv::KalmanFilter _k;
		cv::Mat _state;
		cv::Mat _measurement;
		int _stateTransitionSize = 4;
		int _measurementsSize = 2;
		int _controlSize = 0;
		int _type = CV_32F;
		bool _initialized = false;
		bool _bFullDebugOn = false;
		Max_Size_Vector<cv::Point> _predicted_list;
		Max_Size_Vector<cv::Point> _corrected_list;
		Max_Size_Vector<cv::Point> _GroundTruth_list;
		string _status;

	public:
		kalman();
		kalman(int stateTransitionSize, int measurementSize, cv::Mat A, cv::Mat H, cv::Mat P, cv::Mat Q, cv::Mat R, bool bFullDebudOn);
		kalman(unsigned int iMode, bool bFullDebugOn);

		void Restart() {
			_k.init(_stateTransitionSize, _measurementsSize, _controlSize, _type);
		};

		void SetTransitionMatrix(cv::Mat A);
		void SetMeasurementMatrix(cv::Mat H);
		void SetUncertaintyMatrix(cv::Mat R);
		void SetStateNoiseCovMatrix(cv::Mat Q);
		void SetMeasuresNoiseCovMatrix(cv::Mat R);

		Point Predict(Point ball);

		//  getters for trajectory
		Max_Size_Vector<cv::Point> getPredictedList() const {
			return _predicted_list;
		}

		Max_Size_Vector<cv::Point> getCorrectedList() const {
			return _corrected_list;
		}

		Max_Size_Vector<cv::Point> getGroundTruthList() const {
			return _GroundTruth_list;
		}

		const string& getStatus() const {
			return _status;
		}

		void setStatus(const string &status) {
			this->_status = status;
		}

	};
}

#endif
