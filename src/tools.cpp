#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
	rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
	if(estimations.size()==0){
	  cout<<"Error- empty estimations"<<endl;
	  //return -1;
	}
	if(estimations.size()!=ground_truth.size()){
	  cout<<"Error- estimations and ground truth have different size."<<endl;
	  //return -1;
	}
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
    // ... your code here
    VectorXd residual = estimations[i] -ground_truth[i];
		VectorXd residual_squre = residual.array()*residual.array();
		rmse = rmse + residual_squre;
	}

	//calculate the mean
	// ... your code here
	rmse = rmse/estimations.size();

	//calculate the squared root
	// ... your code here
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 

	//check division by zero
	if(px==0 && py==0){
	  cout<<"Error-divided by zero."<<endl;
	}
	
	//compute the Jacobian matrix
	float tmp = px*px + py*py;
    Hj << px/(sqrt(tmp)),py/(sqrt(tmp)),0,0,
          -py/tmp,px/tmp,0,0,
          py*(vx*py-vy*px)/(tmp*sqrt(tmp)),px*(vy*px-vx*py)/(tmp*sqrt(tmp)),px/sqrt(tmp),py/sqrt(tmp);

	return Hj;
}

