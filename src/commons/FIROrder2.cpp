/*
 * File:   ComauSmartSix.cpp
 * Author: daniele
 *
 * Created on 22 gennaio 2014, 11.43
 */

#include "FIROrder2.h"

namespace lar_comau{

  FIROrder2::FIROrder2(int data_size, double cutoff, double quality) {
      this->data_size = data_size;
      this->Q = quality;
      this->updateParameters(cutoff);

      this->data.resize(data_size);
      this->output.resize(data_size);
      for(int i = 0; i < data_size; i++){
        this->data[i].resize(4);
      }
  }

  FIROrder2::~FIROrder2() {

  }

  void FIROrder2::setSampleTime(double sample_time){
    this->Fs = 1.0 / sample_time;
    this->updateParameters(this->Fc);
  }

  void FIROrder2::updateParameters(double cutoff){
      this->Fc = cutoff;

      this->W = tan(M_PI*this->Fc/this->Fs);
      this->N = 1.0/(pow(this->W,2)+this->W/Q+1);
      this->B0 = this->N*pow(this->W,2);
      this->B1 = 2*this->B0;
      this->B2 = this->B0;
      this->A1 = 2*this->N*(pow(this->W,2)-1);
      this->A2 = this->N*(pow(this->W,2)-this->W/this->Q+1);
  }


  void FIROrder2::setInput(int i , double X){

    double Acc =
      X*this->B0
      +this->data[i][0]*this->B1
      +this->data[i][1]*this->B2
      -this->data[i][2]*this->A1
      -this->data[i][3]*this->A2;

      this->data[i][3]=this->data[i][2];
      this->data[i][2]=Acc;
      this->data[i][1]=this->data[i][0];
      this->data[i][0]=X;

    this->output[i] = Acc;
  }

  void FIROrder2::setInputs(double* Xs){
    for(int i = 0; i < this->data_size ; i++){
        this->setInput(i,Xs[i]);
    }
  }

  void FIROrder2::setInitialData(double* Xs){
    for(int i = 0; i < this->data_size ; i++){
        this->data[i][0]=Xs[i];
        this->data[i][1]=Xs[i];
        this->data[i][2]=Xs[i];
        this->data[i][3]=Xs[i];
        this->output[i] = Xs[i];
    }
  }

  void FIROrder2::setInitialData(float* Xs){
    for(int i = 0; i < this->data_size ; i++){
        this->data[i][0]=Xs[i];
        this->data[i][1]=Xs[i];
        this->data[i][2]=Xs[i];
        this->data[i][3]=Xs[i];
        this->output[i] = Xs[i];
    }
  }


}
