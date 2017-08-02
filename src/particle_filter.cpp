/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *      Edited by : Chris Lawrence July 2017 (CEL)
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// project task: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	//CEL: set number of particles.  More is not better, just need enough.
	num_particles = 200;

	//CEL: resize vectors to fit incoming particle initialization
	particles.resize(num_particles);
	weights.resize(num_particles);

	//CEL: for generating random noise for particle settings
	default_random_engine gen;

	//CEL: Create a normal distribution of the random noise for particle settings
	normal_distribution<double> d_x(x, std[0]);
	normal_distribution<double> d_y(y, std[1]);
	normal_distribution<double> d_theta(theta, std[2]);

	//CEL: create the particles from the normal distribution
	for (int i = 0; i < num_particles; ++i)
	{
		//initialize each particle
		particles[i].id = i;
		particles[i].x = d_x(gen);
		particles[i].y = d_y(gen);
		particles[i].theta = d_theta(gen);
		particles[i].weight = 1.0;
	}

	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// project task: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	//CEL: We need noise to make the system work; when making copies of the best current particles for the next iteration of the process, those copies need variation so that there are options to pick from for the next iteration after that.
	
	//CEL: Random noise for creating particle values
	default_random_engine gen;

	//CEL: pre-calc some values to be used later in translating observations coordinate space
	double vy = velocity / yaw_rate;
	double yd = yaw_rate * delta_t;
	double dv = velocity * delta_t;

	//CEL: loop through particle list
	for (int i = 0; i < num_particles; ++i)
	{
		double n_x;
		double n_y;
		double n_theta;
		
		//CEL: Handle 0 yaw rate differently than non-0; can't divide by 0!
		if (yaw_rate == 0)
		{
			//CEL: predict!
			n_x = particles[i].x + dv * cos(particles[i].theta);//velocity * delta_t
			n_y = particles[i].y + dv * sin(particles[i].theta);//velocity * delta_t
			n_theta = particles[i].theta;	//carry through existing theta due to 0 yaw_rate
		}
		else
		{
			
			//CEL: predict!
			n_x = particles[i].x + vy * (sin(particles[i].theta + yd) - sin(particles[i].theta));	
 			n_y = particles[i].y + vy * (cos(particles[i].theta) - cos(particles[i].theta + yd));
			n_theta = particles[i].theta + yd;
}
		//CEL: add normally distributed noise to the new values
		normal_distribution<double> d_x(n_x, std_pos[0]);
		normal_distribution<double> d_y(n_y, std_pos[1]);
		normal_distribution<double> d_theta(n_theta, std_pos[2]);

		particles[i].x = d_x(gen);
		particles[i].y = d_y(gen);
		particles[i].theta = d_theta(gen);
	}
}

//void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Project task: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	//CEL: did not utilize this due to time
//}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// project task: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	//CEL: pre-calc the Gaussian distribution formula part 1:
	double e1 = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);

	//CEL: pre-calc Gaussian distribution formula parts 2 (demoninators):
	double d0 = 2 * std_landmark[0] * std_landmark[0];
	double d1 = 2 * std_landmark[1] * std_landmark[1];

	//CEL: loop through particles
	for (int i = 0; i < num_particles; ++i) {
		//CEL: initialize the gaussian we will calculate later.
		double gaussian = 1.0;

		//CEL: loop through observations
		for (int q = 0; q < observations.size(); ++q) 
		{
			//rotate the observation to map coords
			double obs_xt = observations[q].x * cos(particles[i].theta) - observations[q].y * sin(particles[i].theta) + particles[i].x;
			double obs_yt = observations[q].x * sin(particles[i].theta) + observations[q].y * cos(particles[i].theta) + particles[i].y;

			//CEL: initialize the minimum distance to closest landmarks
			//CEL: removed due to segfault, replaced with later distance(), min_element() method below
			//double min_distance = 100000000.0;
			//int closest_landmark = -1;

			//CEL: Locate nearest known landmark
			vector<double> landmark_distance (map_landmarks.landmark_list.size());
			for (int p = 0; p < map_landmarks.landmark_list.size(); ++p)
			{
				//CEL: only look at nearby landmarks using a rough rectangle method discussed in class forum
				if (sqrt(pow(particles[i].x - map_landmarks.landmark_list[p].x_f, 2) + pow(particles[i].y - map_landmarks.landmark_list[p].y_f, 2)) <= sensor_range)
				{
					//CEL: calculate the distance from the observed x&y in map coords with the landmarks' x&y values
					landmark_distance[p] = sqrt(pow(obs_xt - map_landmarks.landmark_list[p].x_f, 2) + pow(obs_yt - map_landmarks.landmark_list[p].y_f, 2));
				}
				else
				{
					//CEL: if the landmark is outside the range, set the distance as superfar, aka ignore
					landmark_distance[p] = 10000000.0;
				}
				//CEL: identify the closest landmark, one distance check at a time
				//CEL: replaced with distance() and min_element() below
				//if (landmark_distance[p] < min_distance)
				//{
				//	min_distance = landmark_distance[p];
				//	closest_landmark = map_landmarks.landmark_list[p-1].id_i;
				//}
				
			}
			//CEL: find the minimal distance landmark per point at one time.  Replaces manually tracking lowest value to resolve a strange segfault.
			int closest_landmark = distance(landmark_distance.begin(), min_element(landmark_distance.begin(),landmark_distance.end()));
		
			// CEL: Calculate multi-variate Gaussian distribution
			// 1/(2*pi*sigx*sigy)*e^-((x-ux)^2/(2*sigx^2) + (y-uy)^2/(2*sigy^2))

      			double x_diff = obs_xt - map_landmarks.landmark_list[closest_landmark].x_f;
      			double y_diff = obs_yt - map_landmarks.landmark_list[closest_landmark].y_f;
			gaussian *= e1 * exp(-(((x_diff * x_diff) / d0) + ((y_diff * y_diff) / d1)));
		}
	//CEL: update weights
	particles[i].weight = gaussian;
	weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample() {
	// project task: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	//CEL: set up randomization needed for resampling, and rely on discrete distribution for resampling
	//CEL: using the weights vectors to select for the distribution comes from the Q&A
	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(), weights.end());

	vector<Particle> resampled_particles;

	for (int i = 0; i < num_particles; i++)
	{
		resampled_particles.push_back(particles[distribution(gen)]);
	}
	particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
