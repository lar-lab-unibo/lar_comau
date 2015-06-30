/*
 * File:   ComauSmartSix.h
 * Author: daniele
 *
 * Created on 22 gennaio 2014, 11.43
 */
#include <utility>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <vector>

#ifndef FIRORDER2_H
#define	FIRORDER2_H

namespace lar_comau{
    class FIROrder2 {
      public:
          FIROrder2(int data_size, double cutoff, double quality = 0.5);
          virtual ~FIROrder2();
          void updateParameters(double cutoff);
          void setSampleTime(double sample_time);

          void setInput(int index, double X);
          void setInputs(double* Xs);
          void setInitialData(double* Xs);
          void setInitialData(float* Xs);

          double Fc;
          double Fs;
          double Q;
          double W;
          double N;
          double B0;
          double B1;
          double B2;
          double A1;
          double A2;

          std::vector<double> output;
          std::vector<std::vector<double> > data;

      private:

          int data_size;


    };
}

#endif	/* FIRORDER2_H */
