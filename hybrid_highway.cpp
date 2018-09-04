#include <iostream>
//#include <random>
#include <fstream> 
#include <sstream>
#include <cstdlib>
#include <iomanip>
#include <string>
#include <cstring>
#include <algorithm>
#include <vector>
#include <ctime>
#include <cmath>
#include <unistd.h>
//#define PRINT_PARAMETERS
//#define MULTI
//#define COR_G
//#define PROFILE
//#define TRACE
//#define EX
//#define CPP11
#ifdef MULTI
#include <mpi.h>
#endif

int OPEN=0;
int VDR=0;

#ifdef MULTI
void para_range(int n1, int n2, int &nprocs, int &irank, int &ista, int &iend){
	int iwork1;
	int iwork2;
	iwork1 = ( n2 -n1 + 1 ) / nprocs;
	iwork2 = ( ( n2 -n1 + 1 ) % nprocs);
	ista= irank* iwork1 + n1 + std::min(irank, iwork2);
	iend= ista+ iwork1 -1;
	if ( iwork2 > irank) iend= iend+ 1;
}
#endif

#ifdef CPP11

std::mt19937 mt_rand;
std::uniform_real_distribution<double> rng(0,1);
#endif

struct Vehicle{
	int position, velocity;

	Vehicle(int p, int v) : position(p), velocity(v) {}
};

struct DataSheet{
	double column_1, column_2;
};

struct less_than_position{
	inline bool operator() (const Vehicle& struct1, const Vehicle& struct2){
		return (struct1.position < struct2.position);
	}
};

struct is_Equal {
	int target_;
	is_Equal(int target) : target_(target) {};
	bool operator () (const Vehicle& veh) {
		return veh.position == target_;
	};
};



void printLane(std::vector<Vehicle> &lane){
	std::cout << "Vehicle position:";
	for(int i = 0; i < lane.size(); i++) {
        std::cout << std::setw(3) << lane[i].position;
    }
	std::cout << std::endl;
	
	std::cout << "Vehicle velocity:";
	for(int i = 0; i < lane.size(); i++) {
        std::cout << std::setw(3) << lane[i].velocity;
    }
    std::cout << std::endl;
}

double averageLane(int *speed_of_car, int L){
	
	//Calculating Vavg(t)
	double Vavg=0;
	int N=0;
	for(int i=0; i<L; i++){
		if(speed_of_car[i]>-1){
			Vavg+=speed_of_car[i];
			N++;
		}
	}
	if(N!=0)
		Vavg/=N;
	
	return Vavg;
}

double averageVehType(int *speed_of_car, int *type_of_car, int L, int type){
	
	//Calculating Vavg(t)
	double Vavg=0;
	int N=0;
	for(int i=0; i<L; i++){
		if(type_of_car[i]==type){
			Vavg+=speed_of_car[i];
			N++;
		}
	}
	if(N!=0)
		Vavg/=N;
	
	return Vavg;
}

int numVehiclesLane(int *speed_of_car, int L){
	
	//Calculating N
	int N=0;
	for(int i=0; i<L; i++){
		if(speed_of_car[i]>-1){
			
			N++;
		}
	}
	return N;
}

int numVehType(int *speed_of_car, int *type_of_car, int L, int type){
	
	//Calculating N
	
	int N=0;
	for(int i=0; i<L; i++){
		if(type_of_car[i]==type){
			
			N++;
		}
	}
	
	
	return N;
}

void showhelpinfo(char *s){
	std::cout << "Usage:   " << s <<" [-option] [argument]" << std::endl;
	std::cout << "option:  " << "-h  Show help information" << std::endl;
	std::cout << "         " << "-L  System size" << std::endl;	
	std::cout << "         " << "-p  Braking probability" << std::endl;
	std::cout << "         " << "-s  Timesteps" << std::endl;
	std::cout << "         " << "-f  Fraction of fast cars" << std::endl;
	std::cout << "         " << "-V  Vmax of fast cars" << std::endl;
	std::cout << "         " << "-U  Vmax of slow cars" << std::endl;
	std::cout << "         " << "-l  Number of Lanes" << std::endl;
	std::cout << "         " << "-t  Number of Samples" << std::endl;
	std::cout << "example: " << s << " -L 1000 -p 0.2 -s 100000 -V 2 -l 1 -t 1" << std::endl;
}

void lane_initial(int *position_of_lane, int *speed_of_car, int *type_of_car, int N, int L, int num_fast_veh){
	
	if(N>L)
		N=L;
	if(num_fast_veh>N)
		num_fast_veh=N;
	
	for(int i=0; i<L; i++)
		position_of_lane[i]=i;
		
	//Assigning N different positions.
	std::random_shuffle(position_of_lane, position_of_lane+L);
	
	
	//speed
	for(int i=0; i<L; i++)//clear
		speed_of_car[i]=-1;
				
	for(int i=0; i<N; i++)//Initial speed=0
		speed_of_car[position_of_lane[i]]=0;
	
	//type
	
	int num_slow_veh=N-num_fast_veh;
	int type_arr[N];
	
	for(int i=0; i<num_fast_veh; i++)//arr={0, 0, 0, ...}, vehicle type fast.
		type_arr[i]=0;
		
	for(int i=num_fast_veh; i<N; i++)//arr={1, 1, 1, ...}, vehicle type slow.
		type_arr[i]=1;
		
	std::random_shuffle(type_arr, type_arr+N);//Shuffling type_arr.
	
	for(int i=0; i<L; i++)//clear
		type_of_car[i]=-1;
	
	for(int i=0; i<N; i++)
		type_of_car[position_of_lane[i]]=type_arr[i];
		
	return;
}

void NS(int *speed_of_car, int *type_of_car, int L, int *Vmax, double p, double *p_braking){
	//A1: Acceleration
	for(int i = 0; i < L; i++) {
		if(speed_of_car[i]>=0)
			if(speed_of_car[i]<Vmax[type_of_car[i]])
				speed_of_car[i]+=1;
	}

	//A2: Deceleration (safety distance)
	for(int i = 0; i < L; i++) {
		
		if(speed_of_car[i]<=-1)
			continue;
			
		
		int pos_diff=1;
		while(speed_of_car[(i+pos_diff)%L]<0){
			pos_diff++;
		
		}
		
		if(speed_of_car[i]>pos_diff-1)
			speed_of_car[i]=pos_diff-1;
	}
	
	
	//A3: Random braking:
        if(VDR){
        for(int i = 0; i < L; i++) {
                if(speed_of_car[i]<=0)
                        continue;
                else if(speed_of_car[i]>0 && (double) rand() / (RAND_MAX+1.0) < p_braking[speed_of_car[i]])
                        speed_of_car[i]-=1;
                }
	}
        else{
	for(int i = 0; i < L; i++) {
		if(speed_of_car[i]<=0)
			continue;
		else if(speed_of_car[i]>0 && (double) rand() / (RAND_MAX+1.0) < p)
			speed_of_car[i]-=1;
		}
        }
	
	//A4: Moving
	for(int i = 0, j=0; i < L; i++) {
		if(speed_of_car[i]<=-1)
			continue;
		
		j=speed_of_car[i];
		std::swap(speed_of_car[i], speed_of_car[(i+j)%L]);
		std::swap(type_of_car[i], type_of_car[(i+j)%L]);
		
		i=i+j;

	}
	
	return;
}
void NS_open(int *speed_of_car, int *type_of_car, int L, int *Vmax, double p, double *p_braking, int leave_range, double p_enter, int entering_speed, double p_fast_ratio){
	
	int enter_range=1;
	//A0: Entering
	for(int i=0; i<enter_range; i++)
		if((double) rand() / (RAND_MAX+1.0) < p_enter && speed_of_car[i]==-1){
			speed_of_car[i]=entering_speed;
			if((double) rand() / (RAND_MAX+1.0) < p_fast_ratio)
				type_of_car[i]=0;
			else
				type_of_car[i]=1;
			
		}
			
	
	//A1: Acceleration 
	for(int i = 0; i < L; i++) {
		if(speed_of_car[i]>=0)
			if(speed_of_car[i]<Vmax[type_of_car[i]])
				speed_of_car[i]+=1;
	}

	//A2: Deceleration (safety distance)
	for(int i = 0; i < L; i++) {
		
		if(speed_of_car[i]<=-1)
			continue;
			
		
		int pos_diff=1;
		while(speed_of_car[(i+pos_diff)%L]<0){
			pos_diff++;
		
		}
		
		if(speed_of_car[i]>pos_diff-1)
			speed_of_car[i]=pos_diff-1;
	}
	
	
	//A3: Random braking
	if(VDR){
        for(int i = 0; i < L; i++) {
                if(speed_of_car[i]<=0)
                        continue;
                else if(speed_of_car[i]>0 && (double) rand() / (RAND_MAX+1.0) < p_braking[speed_of_car[i]])
                        speed_of_car[i]-=1;
                }
	}
	else{
	for(int i = 0; i < L; i++) {
		if(speed_of_car[i]<=0)
			continue;
		else if(speed_of_car[i]>0 && (double) rand() / (RAND_MAX+1.0) < p)
			speed_of_car[i]-=1;
		}
	}
	
	//A4: Moving
	for(int i = 0, j=0; i < L; i++) {
		if(speed_of_car[i]<=-1)		// empty
			continue;
		if(L-(i+j)>leave_range){//x = x + v
			j=speed_of_car[i];
			std::swap(speed_of_car[i], speed_of_car[(i+j)%L]);
			std::swap(type_of_car[i], type_of_car[(i+j)%L]);
			i=i+j;
		}
		else{//disappear
			speed_of_car[i]=-1;
			type_of_car[i]=-1;
		}

	}
	
	return;
}


#ifdef TRACE
void NS_part_1(int *speed_of_car, int *type_of_car, int L, int *Vmax, double p, double *p_braking){
	//A1: 
	for(int i = 0; i < L; i++) {
		if(speed_of_car[i]>=0)
			if(speed_of_car[i]<Vmax[type_of_car[i]])
				speed_of_car[i]+=1;
	}

	//A2:
	for(int i = 0; i < L; i++) {
		
		if(speed_of_car[i]<=-1)
			continue;
			
		
		int pos_diff=1;
		while(speed_of_car[(i+pos_diff)%L]<0){
			pos_diff++;
		
		}
		
		if(speed_of_car[i]>pos_diff-1)
			speed_of_car[i]=pos_diff-1;
	}
	
	//A3:
	if(VDR){
        for(int i = 0; i < L; i++) {
                if(speed_of_car[i]<=0)
                        continue;
                else if(speed_of_car[i]>0 && (double) rand() / (RAND_MAX+1.0) < p_braking[speed_of_car[i]])
                        speed_of_car[i]-=1;
                }
	}
	else{
	for(int i = 0; i < L; i++) {
		if(speed_of_car[i]<=0)
			continue;
		else if(speed_of_car[i]>0 && (double) rand() / (RAND_MAX+1.0) < p)
			speed_of_car[i]-=1;
		}
	}
	return;
	
}

void NS_part_2(int *speed_of_car, int *type_of_car, int L, int *Vmax, double p, double *p_braking){
	
	//A4:
	for(int i = 0, j=0; i < L; i++) {
		if(speed_of_car[i]<=-1)
			continue;
		
		j=speed_of_car[i];
		std::swap(speed_of_car[i], speed_of_car[(i+j)%L]);
		std::swap(type_of_car[i], type_of_car[(i+j)%L]);
		
		i=i+j;

	}
	
	return;
}

void NS_open_part_1(int *speed_of_car, int *type_of_car, int L, int *Vmax, double p, double *p_braking, int leave_range, double p_enter, int entering_speed, double p_fast_ratio){
	
	int enter_range=1;
	//A0: Entering
	for(int i=0; i<enter_range; i++)
		if((double) rand() / (RAND_MAX+1.0) < p_enter && speed_of_car[i]==-1){
			speed_of_car[i]=entering_speed;
			if((double) rand() / (RAND_MAX+1.0) < p_fast_ratio)
				type_of_car[i]=0;
			else
				type_of_car[i]=1;
			
		}
			
	
	//A1:
	for(int i = 0; i < L; i++) {
		if(speed_of_car[i]>=0)
			if(speed_of_car[i]<Vmax[type_of_car[i]])
				speed_of_car[i]+=1;
	}

	//A2:
	for(int i = 0; i < L; i++) {
		
		if(speed_of_car[i]<=-1)
			continue;
			
		
		int pos_diff=1;
		while(speed_of_car[(i+pos_diff)%L]<0){
			pos_diff++;
		
		}
		
		if(speed_of_car[i]>pos_diff-1)
			speed_of_car[i]=pos_diff-1;
	}
	
	//A3:
	if(VDR){
        for(int i = 0; i < L; i++) {
                if(speed_of_car[i]<=0)
                        continue;
                else if(speed_of_car[i]>0 && (double) rand() / (RAND_MAX+1.0) < p_braking[speed_of_car[i]])
                        speed_of_car[i]-=1;
                }
	}
	else{
	for(int i = 0; i < L; i++) {
		if(speed_of_car[i]<=0)
			continue;
		else if(speed_of_car[i]>0 && (double) rand() / (RAND_MAX+1.0) < p)
			speed_of_car[i]-=1;
		}
	}
	
	return;
	
	
}
void NS_open_part_2(int *speed_of_car, int *type_of_car, int L, int *Vmax, double p, double *p_braking, int leave_range, double p_enter, int entering_speed, double p_fast_ratio){
	//A4:
	for(int i = 0, j=0; i < L; i++) {
		if(speed_of_car[i]<=-1)		// empty
			continue;
		if(L-(i+j)>leave_range){//x = x + v
			j=speed_of_car[i];
			std::swap(speed_of_car[i], speed_of_car[(i+j)%L]);
			std::swap(type_of_car[i], type_of_car[(i+j)%L]);
			i=i+j;
		}
		else{//disappear
			speed_of_car[i]=-1;
			type_of_car[i]=-1;
		}

	}
	
	return;
}

#endif

void symmetric_decision_stage(int *position_of_lane, int *speed_of_car, int *type_of_car, int L, int num_of_lanes, int *Vmax, double plc, int &left_token, int &right_token, int &lane_token, int &i, int pos_diff_current_front, int pos_diff_right_front, int pos_diff_right_behind, int pos_diff_left_front, int pos_diff_left_behind){
	//right->left
	if(lane_token+1<num_of_lanes
	&& std::min(speed_of_car[L*lane_token+i]+1, Vmax[type_of_car[L*lane_token+i]])>pos_diff_current_front-1 // LC1: the gap in front in the current lane is less than min(v + 1, v_max).
	&& pos_diff_left_front>pos_diff_current_front // LC2: the gap in front in the other lane is larger than the gap available in front in the current lane
	&& speed_of_car[L*(lane_token+1)+i]<0 // LC3: the target site in the other lane is empty.
	&& pos_diff_left_behind-1>=Vmax[0]){ // LC4: the maximum possible speed of the vehicle behind in the other lane is smaller than the gap in between.
		// The vehicle changes lane with probability plc if LC1-LC4 are satisfied
		left_token=1;
	}
	//left->right
	if(lane_token-1>=0
	&& std::min(speed_of_car[L*lane_token+i]+1, Vmax[type_of_car[L*lane_token+i]])>pos_diff_current_front-1// LC1
	&& pos_diff_right_front>pos_diff_current_front // LC2 
	&& speed_of_car[L*(lane_token-1)+i]<0 // LC3 
	&& pos_diff_right_behind-1>=Vmax[0]){ // LC4
		// The vehicle changes lane with probability plc if LC1-LC4 are satisfied
		right_token=1;
	}
}

void symmetric(int *position_of_lane, int *speed_of_car, int *type_of_car, int L, int num_of_lanes, int *Vmax, double plc){
	
	
	for(int i=0; i<num_of_lanes*L; i++)
		position_of_lane[i]=0;
	
	
	for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
		for(int i = 0; i < L; i++){
			if(speed_of_car[L*lane_token+i]<=-1)
				continue;
			
			int pos_diff_current_front=1;
			while(speed_of_car[L*lane_token+(i+pos_diff_current_front)%L]<0 && pos_diff_current_front<L){
				pos_diff_current_front++;
			}
			int pos_diff_right_front=1;
			if(lane_token-1>=0){
				while(speed_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]<0 && pos_diff_right_front<L){
					pos_diff_right_front++;
				}
			}
			int pos_diff_right_behind=1;
			if(lane_token-1>=0){
				while(speed_of_car[L*(lane_token-1)+(i-pos_diff_right_behind+L)%L]<0 && pos_diff_right_behind<L){
					pos_diff_right_behind++;
				}
			}
			int pos_diff_left_front=1;
			if(lane_token+1<num_of_lanes){
				while(speed_of_car[L*(lane_token+1)+(i+pos_diff_left_front)%L]<0 && pos_diff_left_front<L){
					pos_diff_left_front++;
				}
			}
			int pos_diff_left_behind=1;
			if(lane_token+1<num_of_lanes){
				while(speed_of_car[L*(lane_token+1)+(i-pos_diff_left_behind+L)%L]<0 && pos_diff_left_behind<L){
					pos_diff_left_behind++;
				}
			}
			int left_token=0, right_token=0;
			//right->left
			if(lane_token+1<num_of_lanes
			&& std::min(speed_of_car[L*lane_token+i]+1, Vmax[type_of_car[L*lane_token+i]])>pos_diff_current_front-1//the gap in front in the current lane is less than min(v + 1, v_max).
			&& pos_diff_left_front>pos_diff_current_front//the gap in front in the other lane is larger than the gap available in front in the current lane.
			&& speed_of_car[L*(lane_token+1)+i]<0//the nearest-neighbour site in the other lane is empty.
			&& pos_diff_left_behind-1>=Vmax[0]){//the maximum possible speed of the vehicle behind in the other lane is smaller than the gap in between.
				//A vehicle changes lane with probability plc provided the conditions above.
				left_token=1;
			}
			//left->right
			if(lane_token-1>=0
			&& std::min(speed_of_car[L*lane_token+i]+1, Vmax[type_of_car[L*lane_token+i]])>pos_diff_current_front-1//the gap in front in the current lane is less than min(v + 1, v_max).
			&& pos_diff_right_front>pos_diff_current_front//the gap in front in the other lane is larger than the gap available in front in the current lane.
			&& speed_of_car[L*(lane_token-1)+i]<0//the nearest-neighbour site in the other lane is empty.
			&& pos_diff_right_behind-1>=Vmax[0]){//the maximum possible speed of the vehicle behind in the other lane is smaller than the gap in between.
				//A vehicle changes lane with probability plc provided the conditions above.
				right_token=1;
			}
			
			if(left_token==1 && right_token==1){
				if(pos_diff_left_front>pos_diff_right_front){
					if((double) rand() / (RAND_MAX+1.0) < plc)
						position_of_lane[L*lane_token+i]=1;
				}
				else if(pos_diff_left_front<pos_diff_right_front){
					if((double) rand() / (RAND_MAX+1.0) < plc)
						position_of_lane[L*lane_token+i]=2;
				}
				else if(pos_diff_left_front==pos_diff_right_front){
					if((double) rand() / (RAND_MAX+1.0) < 0.5){
						if((double) rand() / (RAND_MAX+1.0) < plc)
							position_of_lane[L*lane_token+i]=1;
					}else{
						if((double) rand() / (RAND_MAX+1.0) < plc)
							position_of_lane[L*lane_token+i]=2;
					}
				}
			}else if(left_token==1){
				if((double) rand() / (RAND_MAX+1.0) < plc)
					position_of_lane[L*lane_token+i]=1;
			}else if(right_token==1){
				if((double) rand() / (RAND_MAX+1.0) < plc)
					position_of_lane[L*lane_token+i]=2;
			}
			
			
			
		}	
	}
		
	for(int lane_token=0; lane_token<num_of_lanes-2; lane_token++){//To prevent collision.
		for(int i = 0; i < L; i++){
			if(position_of_lane[L*lane_token+i]==1 && position_of_lane[L*(lane_token+2)+i]==2){
				position_of_lane[L*lane_token+i]=0;
				position_of_lane[L*(lane_token+2)+i]=0;
			}
		}
	}
	
	
	for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
		for(int i=0; i<L; i++){
			if(lane_token+1<num_of_lanes && position_of_lane[L*lane_token+i]==1){//right->left
				std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token+1)+i]);
				std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token+1)+i]);
			}
			else if(lane_token-1>=0 && position_of_lane[L*lane_token+i]==2){//left->right
				std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token-1)+i]);
				std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token-1)+i]);
			}
		}
	}
	
}
void asymmetric_decision_stage(int *position_of_lane, int *speed_of_car, int *type_of_car, int L, int num_of_lanes, int *Vmax, double plc, int &left_token, int &right_token, int &lane_token, int &i, int pos_diff_current_front, int pos_diff_right_front, int pos_diff_right_behind, int pos_diff_left_front, int pos_diff_left_behind){
	//right->left
	if(lane_token+1<num_of_lanes
	&& (std::min(speed_of_car[L*lane_token+i]+1, Vmax[type_of_car[L*lane_token+i]])>pos_diff_current_front-1 // LC1: the gap in front in the current lane is less than min(v + 1, v_max), 
		|| (type_of_car[L*lane_token+i]==0 && // LC2': the vehicle under consideration, which is of the "fast" type, 
			type_of_car[L*lane_token+(i+pos_diff_current_front)%L]==1 && //finds a vehicle of the "slow" type 
			Vmax[0]>pos_diff_current_front-1))//at a distance less than Vfmax in front of it. (A fast vehicle change lane in advance when behind a slow vehicle)
	&&(pos_diff_left_front>pos_diff_current_front)// LC2: the gap in front in the left lane is larger than the gap available in front in the right lane.
	&&(speed_of_car[L*(lane_token+1)+i]<0)// LC3: the target site in the left lane is empty.
	&&(pos_diff_left_behind-1>=Vmax[0])){// LC4: the maximum possible speed of the vehicle behind in the left lane is smaller than the gap in between.
		if((double) rand() / (RAND_MAX+1.0) < plc)// The vehicle changes lane with probability plc if LC1-LC4 are satisfied.
			left_token=1;
	}
	//left->right
	else if(lane_token-1>=0
		&& (pos_diff_right_front-1>=speed_of_car[L*lane_token+i]) 
		&& (speed_of_car[L*(lane_token-1)+i]<0) // LC3: the target site in the right lane is empty.
		&& (pos_diff_right_behind-1>=Vmax[0])// LC4: the maximum possible speed of the vehicle behind in the right lane is smaller than the gap in between.
		&& (type_of_car[L*lane_token+i]==1 ||//a fast vehicle does not shift from the left to the right lane if it finds a vehicle of "slow" type in front in the right lane within a distance of Vfmax.
			(type_of_car[L*lane_token+i]==0 && !(type_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]==1 && Vmax[0]>pos_diff_right_front-1))
			)){
				if((double) rand() / (RAND_MAX+1.0) < plc)// The vehicle changes lane with probability plc provided the conditions above.
					right_token=1;
	}
}
void asymmetric(int *position_of_lane, int *speed_of_car, int *type_of_car, int L, int num_of_lanes, int *Vmax, double plc){

	for(int i=0; i<num_of_lanes*L; i++)
		position_of_lane[i]=0;
	
	
	for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
		for(int i = 0; i < L; i++){
			if(speed_of_car[L*lane_token+i]<=-1)
				continue;
			
			int pos_diff_current_front=1;
			while(speed_of_car[L*lane_token+(i+pos_diff_current_front)%L]<0 && pos_diff_current_front<L){
				pos_diff_current_front++;
			}
			
			int pos_diff_right_front=1;
			if(lane_token-1>=0){
				while(speed_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]<0 && pos_diff_right_front<L){
					pos_diff_right_front++;
				}
			}
			int pos_diff_right_behind=1;
			if(lane_token-1>=0){
				while(speed_of_car[L*(lane_token-1)+(i-pos_diff_right_behind+L)%L]<0 && pos_diff_right_behind<L){
					pos_diff_right_behind++;
				}
			}
			int pos_diff_left_front=1;
			if(lane_token+1<num_of_lanes){
				while(speed_of_car[L*(lane_token+1)+(i+pos_diff_left_front)%L]<0 && pos_diff_left_front<L){
					pos_diff_left_front++;
				}
			}
			int pos_diff_left_behind=1;
			if(lane_token+1<num_of_lanes){
				while(speed_of_car[L*(lane_token+1)+(i-pos_diff_left_behind+L)%L]<0 && pos_diff_left_behind<L){
					pos_diff_left_behind++;
				}
			}
			
			//right->left
			if(lane_token+1<num_of_lanes
			&& (std::min(speed_of_car[L*lane_token+i]+1, Vmax[type_of_car[L*lane_token+i]])>pos_diff_current_front-1 //either the gap in front in the current lane is less than min(v + 1, v_max), 
				|| (type_of_car[L*lane_token+i]==0 && //or, the vehicle under consideration, which is of the "fast" type, 
					type_of_car[L*lane_token+(i+pos_diff_current_front)%L]==1 && //finds a vehicle of the "slow" type 
					Vmax[0]>pos_diff_current_front-1))//at a distance less than Vfmax in front of it. (A fast vehicle change lane in advance when behind a slow vehicle)
			&&(pos_diff_left_front>pos_diff_current_front)//the gap in front in the left lane is larger than the gap available in front in the right lane.
			&&(speed_of_car[L*(lane_token+1)+i]<0)//the nearest-neighbour site in the left lane is empty.
			&&(pos_diff_left_behind-1>=Vmax[0])){//the maximum possible speed of the vehicle behind in the left lane is smaller than the gap in between.
				if((double) rand() / (RAND_MAX+1.0) < plc)//A vehicle changes lane with probability plc provided the conditions above.
					position_of_lane[L*lane_token+i]=1;
			}
			//left->right
			else if(lane_token-1>=0
				&& (pos_diff_right_front-1>=speed_of_car[L*lane_token+i])//the gap in front in the right lane is larger than the current speed of the vehicle in the left lane.
				&& (speed_of_car[L*(lane_token-1)+i]<0)//the nearest-neighbour site in the right lane is empty.
				&& (pos_diff_right_behind-1>=Vmax[0])//the maximum possible speed of the vehicle behind in the right lane is smaller than the gap in between.
				&& (type_of_car[L*lane_token+i]==1 ||//a fast vehicle does not shift from the left to the right lane if it finds a vehicle of "slow" type in front in the right lane within a distance of Vfmax.
					(type_of_car[L*lane_token+i]==0 && !(type_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]==1 && Vmax[0]>pos_diff_right_front-1))
					)){
						if((double) rand() / (RAND_MAX+1.0) < plc)//A vehicle changes lane with probability plc provided the conditions above.
							position_of_lane[L*lane_token+i]=2;
			}
			
		}	
	}
	for(int lane_token=0; lane_token<num_of_lanes-2; lane_token++){//To prevent collision.
		for(int i = 0; i < L; i++){
			if(position_of_lane[L*lane_token+i]==1 && position_of_lane[L*(lane_token+2)+i]==2){
				position_of_lane[L*lane_token+i]=0;
				position_of_lane[L*(lane_token+2)+i]=0;
			}
		}
	}
	
	
	for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
		for(int i=0; i<L; i++){
			if(lane_token+1<num_of_lanes && position_of_lane[L*lane_token+i]==1){//right->left
				std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token+1)+i]);
				std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token+1)+i]);
			}
			else if(lane_token-1>=0 && position_of_lane[L*lane_token+i]==2){//left->right
				std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token-1)+i]);
				std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token-1)+i]);
			}
		}
	}
	
}



void hybrid(int *position_of_lane, int *speed_of_car, int *type_of_car, int L, int num_of_lanes, int *Vmax, double plc){

	for(int i=0; i<num_of_lanes*L; i++)
		position_of_lane[i]=0;
	
	
	for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
		for(int i = 0; i < L; i++){
			if(speed_of_car[L*lane_token+i]<=-1)
				continue;
			
			int pos_diff_current_front=1;
			while(speed_of_car[L*lane_token+(i+pos_diff_current_front)%L]<0 && pos_diff_current_front<L){
				pos_diff_current_front++;
			}
			
			int pos_diff_right_front=1;
			if(lane_token-1>=0){
				while(speed_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]<0 && pos_diff_right_front<L){
					pos_diff_right_front++;
				}
			}
			int pos_diff_right_behind=1;
			if(lane_token-1>=0){
				while(speed_of_car[L*(lane_token-1)+(i-pos_diff_right_behind+L)%L]<0 && pos_diff_right_behind<L){
					pos_diff_right_behind++;
				}
			}
			int pos_diff_left_front=1;
			if(lane_token+1<num_of_lanes){
				while(speed_of_car[L*(lane_token+1)+(i+pos_diff_left_front)%L]<0 && pos_diff_left_front<L){
					pos_diff_left_front++;
				}
			}
			int pos_diff_left_behind=1;
			if(lane_token+1<num_of_lanes){
				while(speed_of_car[L*(lane_token+1)+(i-pos_diff_left_behind+L)%L]<0 && pos_diff_left_behind<L){
					pos_diff_left_behind++;
				}
			}
			
			
			if(lane_token+1==num_of_lanes){//asy
				//right->left
				if(lane_token+1<num_of_lanes
				&& (std::min(speed_of_car[L*lane_token+i]+1, Vmax[type_of_car[L*lane_token+i]])>pos_diff_current_front-1 //either the gap in front in the current lane is less than min(v + 1, v_max), 
					|| (type_of_car[L*lane_token+i]==0 && //or, the vehicle under consideration, which is of the "fast" type, 
						type_of_car[L*lane_token+(i+pos_diff_current_front)%L]==1 && //finds a vehicle of the "slow" type 
						Vmax[0]>pos_diff_current_front-1))//at a distance less than Vfmax in front of it. (A fast vehicle change lane in advance when behind a slow vehicle)
				&&(pos_diff_left_front>pos_diff_current_front)//the gap in front in the left lane is larger than the gap available in front in the right lane.
				&&(speed_of_car[L*(lane_token+1)+i]<0)//the nearest-neighbour site in the left lane is empty.
				&&(pos_diff_left_behind-1>=Vmax[0])){//the maximum possible speed of the vehicle behind in the left lane is smaller than the gap in between.
					if((double) rand() / (RAND_MAX+1.0) < plc)//A vehicle changes lane with probability plc provided the conditions above.
						position_of_lane[L*lane_token+i]=1;
				}
				//left->right
				else if(lane_token-1>=0
					&& (pos_diff_right_front-1>=speed_of_car[L*lane_token+i])//the gap in front in the right lane is larger than the current speed of the vehicle in the left lane.
					&& (speed_of_car[L*(lane_token-1)+i]<0)//the nearest-neighbour site in the right lane is empty.
					&& (pos_diff_right_behind-1>=Vmax[0])//the maximum possible speed of the vehicle behind in the right lane is smaller than the gap in between.
					&& (type_of_car[L*lane_token+i]==1 ||//a fast vehicle does not shift from the left to the right lane if it finds a vehicle of "slow" type in front in the right lane within a distance of Vfmax.
						(type_of_car[L*lane_token+i]==0 && !(type_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]==1 && Vmax[0]>pos_diff_right_front-1))
						)){
							if((double) rand() / (RAND_MAX+1.0) < plc)//A vehicle changes lane with probability plc provided the conditions above.
								position_of_lane[L*lane_token+i]=2;
				}
			}
			else{//sy
				int left_token=0, right_token=0;
				//right->left
				if(lane_token+1<num_of_lanes
				&& std::min(speed_of_car[L*lane_token+i]+1, Vmax[type_of_car[L*lane_token+i]])>pos_diff_current_front-1//the gap in front in the current lane is less than min(v + 1, v_max).
				&& pos_diff_left_front>pos_diff_current_front//the gap in front in the other lane is larger than the gap available in front in the current lane.
				&& speed_of_car[L*(lane_token+1)+i]<0//the nearest-neighbour site in the other lane is empty.
				&& pos_diff_left_behind-1>=Vmax[0]){//the maximum possible speed of the vehicle behind in the other lane is smaller than the gap in between.
					//A vehicle changes lane with probability plc provided the conditions above.
					left_token=1;
				}
				//left->right
				if(lane_token-1>=0
				&& std::min(speed_of_car[L*lane_token+i]+1, Vmax[type_of_car[L*lane_token+i]])>pos_diff_current_front-1//the gap in front in the current lane is less than min(v + 1, v_max).
				&& pos_diff_right_front>pos_diff_current_front//the gap in front in the other lane is larger than the gap available in front in the current lane.
				&& speed_of_car[L*(lane_token-1)+i]<0//the nearest-neighbour site in the other lane is empty.
				&& pos_diff_right_behind-1>=Vmax[0]){//the maximum possible speed of the vehicle behind in the other lane is smaller than the gap in between.
					//A vehicle changes lane with probability plc provided the conditions above.
					right_token=1;
				}
				
				if(left_token==1 && right_token==1){
					if(pos_diff_left_front>pos_diff_right_front){
						if((double) rand() / (RAND_MAX+1.0) < plc)
							position_of_lane[L*lane_token+i]=1;
					}
					else if(pos_diff_left_front<pos_diff_right_front){
						if((double) rand() / (RAND_MAX+1.0) < plc)
							position_of_lane[L*lane_token+i]=2;
					}
					else if(pos_diff_left_front==pos_diff_right_front){
						if((double) rand() / (RAND_MAX+1.0) < 0.5){
							if((double) rand() / (RAND_MAX+1.0) < plc)
								position_of_lane[L*lane_token+i]=1;
						}else{
							if((double) rand() / (RAND_MAX+1.0) < plc)
								position_of_lane[L*lane_token+i]=2;
						}
					}
				}else if(left_token==1){
					if((double) rand() / (RAND_MAX+1.0) < plc)
						position_of_lane[L*lane_token+i]=1;
				}else if(right_token==1){
					if((double) rand() / (RAND_MAX+1.0) < plc)
						position_of_lane[L*lane_token+i]=2;
				}
			}
			
		}	
	}
	for(int lane_token=0; lane_token<num_of_lanes-2; lane_token++){//To prevent collision.
		for(int i = 0; i < L; i++){
			if(position_of_lane[L*lane_token+i]==1 && position_of_lane[L*(lane_token+2)+i]==2){
				position_of_lane[L*lane_token+i]=0;
				position_of_lane[L*(lane_token+2)+i]=0;
			}
		}
	}
	
	
	for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
		for(int i=0; i<L; i++){
			if(lane_token+1<num_of_lanes && position_of_lane[L*lane_token+i]==1){//right->left
				std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token+1)+i]);
				std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token+1)+i]);
			}
			else if(lane_token-1>=0 && position_of_lane[L*lane_token+i]==2){//left->right
				std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token-1)+i]);
				std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token-1)+i]);
			}
		}
	}
	
}


void syoffset(int *position_of_lane, int *speed_of_car, int *type_of_car, int L, int num_of_lanes, int *Vmax, double plc){
	
	
	for(int i=0; i<num_of_lanes*L; i++)
		position_of_lane[i]=0;
	
	
	for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
		for(int i = 0; i < L; i++) {
			if(speed_of_car[L*lane_token+i]<=-1)
				continue;
			
			int pos_diff_current_front=1;
			while(speed_of_car[L*lane_token+(i+pos_diff_current_front)%L]<0 && pos_diff_current_front<L){
				pos_diff_current_front++;
			}
			int pos_diff_right_front=1;
			if(lane_token-1>=0){
				while(speed_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]<0 && pos_diff_right_front<L){
					pos_diff_right_front++;
				}
			}
			int pos_diff_right_behind=1;
			if(lane_token-1>=0){
				while(speed_of_car[L*(lane_token-1)+(i-pos_diff_right_behind+L)%L]<0 && pos_diff_right_behind<L){
					pos_diff_right_behind++;
				}
			}
			int pos_diff_left_front=1;
			if(lane_token+1<num_of_lanes){
				while(speed_of_car[L*(lane_token+1)+(i+pos_diff_left_front)%L]<0 && pos_diff_left_front<L){
					pos_diff_left_front++;
				}
			}
			int pos_diff_left_behind=1;
			if(lane_token+1<num_of_lanes){
				while(speed_of_car[L*(lane_token+1)+(i-pos_diff_left_behind+L)%L]<0 && pos_diff_left_behind<L){
					pos_diff_left_behind++;
				}
			}
			int left_token=0, right_token=0;
			//right->left
			if(lane_token+1<num_of_lanes
			&& speed_of_car[L*lane_token+i]+1>pos_diff_current_front-1//the gap in front in the current lane is less than v + 1.
			&& pos_diff_left_front>pos_diff_current_front//the gap in front in the other lane is larger than the gap available in front in the current lane.
			&& speed_of_car[L*(lane_token+1)+i]<0//the nearest-neighbour site in the other lane is empty.
			&& pos_diff_left_behind-1>=Vmax[0]){//the maximum possible speed of the vehicle behind in the other lane is smaller than the gap in between.
				//A vehicle changes lane with probability plc provided the conditions above.
				left_token=1;
			}
			//left->right
			if(lane_token-1>=0
			&& speed_of_car[L*lane_token+i]+1>pos_diff_current_front-1//the gap in front in the current lane is less than v + 1.
			&& pos_diff_right_front>pos_diff_current_front//the gap in front in the other lane is larger than the gap available in front in the current lane.
			&& speed_of_car[L*(lane_token-1)+i]<0//the nearest-neighbour site in the other lane is empty.
			&& pos_diff_right_behind-1>=Vmax[0]){//the maximum possible speed of the vehicle behind in the other lane is smaller than the gap in between.
				//A vehicle changes lane with probability plc provided the conditions above.
				right_token=1;
			}
			
			if(left_token==1 && right_token==1){
				if(pos_diff_left_front>pos_diff_right_front){
					if((double) rand() / (RAND_MAX+1.0) < plc)
						position_of_lane[L*lane_token+i]=1;
				}
				else if(pos_diff_left_front<pos_diff_right_front){
					if((double) rand() / (RAND_MAX+1.0) < plc)
						position_of_lane[L*lane_token+i]=2;
				}
				else if(pos_diff_left_front==pos_diff_right_front){
					if((double) rand() / (RAND_MAX+1.0) < 0.5){
						if((double) rand() / (RAND_MAX+1.0) < plc)
							position_of_lane[L*lane_token+i]=1;
					}else{
						if((double) rand() / (RAND_MAX+1.0) < plc)
							position_of_lane[L*lane_token+i]=2;
					}
				}
			}else if(left_token==1){
				if((double) rand() / (RAND_MAX+1.0) < plc)
					position_of_lane[L*lane_token+i]=1;
			}else if(right_token==1){
				if((double) rand() / (RAND_MAX+1.0) < plc)
					position_of_lane[L*lane_token+i]=2;
			}
			
			
			
		}	
	}
		
	for(int lane_token=0; lane_token<num_of_lanes-2; lane_token++){//To prevent collision.
		for(int i = 0; i < L; i++){
			if(position_of_lane[L*lane_token+i]==1 && position_of_lane[L*(lane_token+2)+i]==2){
				position_of_lane[L*lane_token+i]=0;
				position_of_lane[L*(lane_token+2)+i]=0;
			}
		}
	}
	
	
	for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
		for(int i=0; i<L; i++){
			if(lane_token+1<num_of_lanes && position_of_lane[L*lane_token+i]==1){//right->left
				std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token+1)+i]);
				std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token+1)+i]);
			}
			else if(lane_token-1>=0 && position_of_lane[L*lane_token+i]==2){//left->right
				std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token-1)+i]);
				std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token-1)+i]);
			}
		}
	}
	
}

void asyoffset(int *position_of_lane, int *speed_of_car, int *type_of_car, int L, int num_of_lanes, int *Vmax, double plc){

	for(int i=0; i<num_of_lanes*L; i++)
		position_of_lane[i]=0;
	
	
	for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
		for(int i = 0; i < L; i++){
			if(speed_of_car[L*lane_token+i]<=-1)
				continue;
			
			int pos_diff_current_front=1;
			while(speed_of_car[L*lane_token+(i+pos_diff_current_front)%L]<0 && pos_diff_current_front<L){
				pos_diff_current_front++;
			}
			
			int pos_diff_right_front=1;
			if(lane_token-1>=0){
				while(speed_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]<0 && pos_diff_right_front<L){
					pos_diff_right_front++;
				}
			}
			int pos_diff_right_behind=1;
			if(lane_token-1>=0){
				while(speed_of_car[L*(lane_token-1)+(i-pos_diff_right_behind+L)%L]<0 && pos_diff_right_behind<L){
					pos_diff_right_behind++;
				}
			}
			int pos_diff_left_front=1;
			if(lane_token+1<num_of_lanes){
				while(speed_of_car[L*(lane_token+1)+(i+pos_diff_left_front)%L]<0 && pos_diff_left_front<L){
					pos_diff_left_front++;
				}
			}
			int pos_diff_left_behind=1;
			if(lane_token+1<num_of_lanes){
				while(speed_of_car[L*(lane_token+1)+(i-pos_diff_left_behind+L)%L]<0 && pos_diff_left_behind<L){
					pos_diff_left_behind++;
				}
			}
			
			//right->left
			if(lane_token+1<num_of_lanes
			&& (speed_of_car[L*lane_token+i]+1>pos_diff_current_front-1 //either the gap in front in the current lane is less than v + 1, 
				|| (type_of_car[L*lane_token+i]==0 && //or, the vehicle under consideration, which is of the "fast" type, 
					type_of_car[L*lane_token+(i+pos_diff_current_front)%L]==1 && //finds a vehicle of the "slow" type 
					Vmax[0]>pos_diff_current_front-1))//at a distance less than Vfmax in front of it. (A fast vehicle change lane in advance when behind a slow vehicle)
			&&(pos_diff_left_front>pos_diff_current_front)//the gap in front in the left lane is larger than the gap available in front in the right lane.
			&&(speed_of_car[L*(lane_token+1)+i]<0)//the nearest-neighbour site in the left lane is empty.
			&&(pos_diff_left_behind-1>=Vmax[0])){//the maximum possible speed of the vehicle behind in the left lane is smaller than the gap in between.
				if((double) rand() / (RAND_MAX+1.0) < plc)//A vehicle changes lane with probability plc provided the conditions above.
					position_of_lane[L*lane_token+i]=1;
			}
			//left->right
			else if(lane_token-1>=0
				&& (pos_diff_right_front-1>=speed_of_car[L*lane_token+i])//the gap in front in the right lane is larger than the current speed of the vehicle in the left lane.
				&& (speed_of_car[L*(lane_token-1)+i]<0)//the nearest-neighbour site in the right lane is empty.
				&& (pos_diff_right_behind-1>=Vmax[0])//the maximum possible speed of the vehicle behind in the right lane is smaller than the gap in between.
				&& (type_of_car[L*lane_token+i]==1 ||//a fast vehicle does not shift from the left to the right lane if it finds a vehicle of "slow" type in front in the right lane within a distance of Vfmax.
					(type_of_car[L*lane_token+i]==0 && !(type_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]==1 && Vmax[0]>pos_diff_right_front-1))
					)){
						if((double) rand() / (RAND_MAX+1.0) < plc)//A vehicle changes lane with probability plc provided the conditions above.
							position_of_lane[L*lane_token+i]=2;
			}
			
		}	
	}
	for(int lane_token=0; lane_token<num_of_lanes-2; lane_token++){//To prevent collision.
		for(int i = 0; i < L; i++){
			if(position_of_lane[L*lane_token+i]==1 && position_of_lane[L*(lane_token+2)+i]==2){
				position_of_lane[L*lane_token+i]=0;
				position_of_lane[L*(lane_token+2)+i]=0;
			}
		}
	}
	
	
	for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
		for(int i=0; i<L; i++){
			if(lane_token+1<num_of_lanes && position_of_lane[L*lane_token+i]==1){//right->left
				std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token+1)+i]);
				std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token+1)+i]);
			}
			else if(lane_token-1>=0 && position_of_lane[L*lane_token+i]==2){//left->right
				std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token-1)+i]);
				std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token-1)+i]);
			}
		}
	}
	
}
void comoffset(int *position_of_lane, int *speed_of_car, int *type_of_car, int L, int num_of_lanes, int *Vmax, double plc){

	for(int i=0; i<num_of_lanes*L; i++)
		position_of_lane[i]=0;
	
	
	for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
		for(int i = 0; i < L; i++){
			if(speed_of_car[L*lane_token+i]<=-1)
				continue;
			
			int pos_diff_current_front=1;
			while(speed_of_car[L*lane_token+(i+pos_diff_current_front)%L]<0 && pos_diff_current_front<L){
				pos_diff_current_front++;
			}
			
			int pos_diff_right_front=1;
			if(lane_token-1>=0){
				while(speed_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]<0 && pos_diff_right_front<L){
					pos_diff_right_front++;
				}
			}
			int pos_diff_right_behind=1;
			if(lane_token-1>=0){
				while(speed_of_car[L*(lane_token-1)+(i-pos_diff_right_behind+L)%L]<0 && pos_diff_right_behind<L){
					pos_diff_right_behind++;
				}
			}
			int pos_diff_left_front=1;
			if(lane_token+1<num_of_lanes){
				while(speed_of_car[L*(lane_token+1)+(i+pos_diff_left_front)%L]<0 && pos_diff_left_front<L){
					pos_diff_left_front++;
				}
			}
			int pos_diff_left_behind=1;
			if(lane_token+1<num_of_lanes){
				while(speed_of_car[L*(lane_token+1)+(i-pos_diff_left_behind+L)%L]<0 && pos_diff_left_behind<L){
					pos_diff_left_behind++;
				}
			}
			
			
			if(lane_token+1==num_of_lanes){//asy
				//right->left
				if(lane_token+1<num_of_lanes
				&& (speed_of_car[L*lane_token+i]+1>pos_diff_current_front-1 //either the gap in front in the current lane is less than v + 1, 
					|| (type_of_car[L*lane_token+i]==0 && //or, the vehicle under consideration, which is of the "fast" type, 
						type_of_car[L*lane_token+(i+pos_diff_current_front)%L]==1 && //finds a vehicle of the "slow" type 
						Vmax[0]>pos_diff_current_front-1))//at a distance less than Vfmax in front of it. (A fast vehicle change lane in advance when behind a slow vehicle)
				&&(pos_diff_left_front>pos_diff_current_front)//the gap in front in the left lane is larger than the gap available in front in the right lane.
				&&(speed_of_car[L*(lane_token+1)+i]<0)//the nearest-neighbour site in the left lane is empty.
				&&(pos_diff_left_behind-1>=Vmax[0])){//the maximum possible speed of the vehicle behind in the left lane is smaller than the gap in between.
					if((double) rand() / (RAND_MAX+1.0) < plc)//A vehicle changes lane with probability plc provided the conditions above.
						position_of_lane[L*lane_token+i]=1;
				}
				//left->right
				else if(lane_token-1>=0
					&& (pos_diff_right_front-1>=speed_of_car[L*lane_token+i])//the gap in front in the right lane is larger than the current speed of the vehicle in the left lane.
					&& (speed_of_car[L*(lane_token-1)+i]<0)//the nearest-neighbour site in the right lane is empty.
					&& (pos_diff_right_behind-1>=Vmax[0])//the maximum possible speed of the vehicle behind in the right lane is smaller than the gap in between.
					&& (type_of_car[L*lane_token+i]==1 ||//a fast vehicle does not shift from the left to the right lane if it finds a vehicle of "slow" type in front in the right lane within a distance of Vfmax.
						(type_of_car[L*lane_token+i]==0 && !(type_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]==1 && Vmax[0]>pos_diff_right_front-1))
						)){
							if((double) rand() / (RAND_MAX+1.0) < plc)//A vehicle changes lane with probability plc provided the conditions above.
								position_of_lane[L*lane_token+i]=2;
				}
			}
			else{//sy
				int left_token=0, right_token=0;
				//right->left
				if(lane_token+1<num_of_lanes
				&& speed_of_car[L*lane_token+i]+1>pos_diff_current_front-1//the gap in front in the current lane is less than v + 1.
				&& pos_diff_left_front>pos_diff_current_front//the gap in front in the other lane is larger than the gap available in front in the current lane.
				&& speed_of_car[L*(lane_token+1)+i]<0//the nearest-neighbour site in the other lane is empty.
				&& pos_diff_left_behind-1>=Vmax[0]){//the maximum possible speed of the vehicle behind in the other lane is smaller than the gap in between.
					//A vehicle changes lane with probability plc provided the conditions above.
					left_token=1;
				}
				//left->right
				if(lane_token-1>=0
				&& speed_of_car[L*lane_token+i]+1>pos_diff_current_front-1//the gap in front in the current lane is less than v + 1.
				&& pos_diff_right_front>pos_diff_current_front//the gap in front in the other lane is larger than the gap available in front in the current lane.
				&& speed_of_car[L*(lane_token-1)+i]<0//the nearest-neighbour site in the other lane is empty.
				&& pos_diff_right_behind-1>=Vmax[0]){//the maximum possible speed of the vehicle behind in the other lane is smaller than the gap in between.
					//A vehicle changes lane with probability plc provided the conditions above.
					right_token=1;
				}
				
				if(left_token==1 && right_token==1){
					if(pos_diff_left_front>pos_diff_right_front){
						if((double) rand() / (RAND_MAX+1.0) < plc)
							position_of_lane[L*lane_token+i]=1;
					}
					else if(pos_diff_left_front<pos_diff_right_front){
						if((double) rand() / (RAND_MAX+1.0) < plc)
							position_of_lane[L*lane_token+i]=2;
					}
					else if(pos_diff_left_front==pos_diff_right_front){
						if((double) rand() / (RAND_MAX+1.0) < 0.5){
							if((double) rand() / (RAND_MAX+1.0) < plc)
								position_of_lane[L*lane_token+i]=1;
						}else{
							if((double) rand() / (RAND_MAX+1.0) < plc)
								position_of_lane[L*lane_token+i]=2;
						}
					}
				}else if(left_token==1){
					if((double) rand() / (RAND_MAX+1.0) < plc)
						position_of_lane[L*lane_token+i]=1;
				}else if(right_token==1){
					if((double) rand() / (RAND_MAX+1.0) < plc)
						position_of_lane[L*lane_token+i]=2;
				}
			}
			
		}	
	}
	for(int lane_token=0; lane_token<num_of_lanes-2; lane_token++){//To prevent collision.
		for(int i = 0; i < L; i++){
			if(position_of_lane[L*lane_token+i]==1 && position_of_lane[L*(lane_token+2)+i]==2){
				position_of_lane[L*lane_token+i]=0;
				position_of_lane[L*(lane_token+2)+i]=0;
			}
		}
	}
	
	
	for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
		for(int i=0; i<L; i++){
			if(lane_token+1<num_of_lanes && position_of_lane[L*lane_token+i]==1){//right->left
				std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token+1)+i]);
				std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token+1)+i]);
			}
			else if(lane_token-1>=0 && position_of_lane[L*lane_token+i]==2){//left->right
				std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token-1)+i]);
				std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token-1)+i]);
			}
		}
	}
	
}

void fssa(int *position_of_lane, int *speed_of_car, int *type_of_car, int L, int num_of_lanes, int *Vmax, double plc){

	for(int i=0; i<num_of_lanes*L; i++)
		position_of_lane[i]=0;
	
	
	for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
		for(int i = 0; i < L; i++){
			if(speed_of_car[L*lane_token+i]<=-1)
				continue;
			
			int pos_diff_current_front=1;
			while(speed_of_car[L*lane_token+(i+pos_diff_current_front)%L]<0 && pos_diff_current_front<L){
				pos_diff_current_front++;
			}
			
			int pos_diff_right_front=1;
			if(lane_token-1>=0){
				while(speed_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]<0 && pos_diff_right_front<L){
					pos_diff_right_front++;
				}
			}
			int pos_diff_right_behind=1;
			if(lane_token-1>=0){
				while(speed_of_car[L*(lane_token-1)+(i-pos_diff_right_behind+L)%L]<0 && pos_diff_right_behind<L){
					pos_diff_right_behind++;
				}
			}
			int pos_diff_left_front=1;
			if(lane_token+1<num_of_lanes){
				while(speed_of_car[L*(lane_token+1)+(i+pos_diff_left_front)%L]<0 && pos_diff_left_front<L){
					pos_diff_left_front++;
				}
			}
			int pos_diff_left_behind=1;
			if(lane_token+1<num_of_lanes){
				while(speed_of_car[L*(lane_token+1)+(i-pos_diff_left_behind+L)%L]<0 && pos_diff_left_behind<L){
					pos_diff_left_behind++;
				}
			}
			
			
			if(type_of_car[L*lane_token+i]==1){//Asy. for slow type
				//right->left
				if(lane_token+1<num_of_lanes
				&& (std::min(speed_of_car[L*lane_token+i]+1, Vmax[type_of_car[L*lane_token+i]])>pos_diff_current_front-1 //either the gap in front in the current lane is less than v + 1, 
					|| (type_of_car[L*lane_token+i]==0 && //or, the vehicle under consideration, which is of the "fast" type, 
						type_of_car[L*lane_token+(i+pos_diff_current_front)%L]==1 && //finds a vehicle of the "slow" type 
						Vmax[0]>pos_diff_current_front-1))//at a distance less than Vfmax in front of it. (A fast vehicle change lane in advance when behind a slow vehicle)
				&&(pos_diff_left_front>pos_diff_current_front)//the gap in front in the left lane is larger than the gap available in front in the right lane.
				&&(speed_of_car[L*(lane_token+1)+i]<0)//the nearest-neighbour site in the left lane is empty.
				&&(pos_diff_left_behind-1>=Vmax[0])){//the maximum possible speed of the vehicle behind in the left lane is smaller than the gap in between.
					if((double) rand() / (RAND_MAX+1.0) < plc)//A vehicle changes lane with probability plc provided the conditions above.
						position_of_lane[L*lane_token+i]=1;
				}
				//left->right
				else if(lane_token-1>=0
					&& (pos_diff_right_front-1>=speed_of_car[L*lane_token+i])//the gap in front in the right lane is larger than the current speed of the vehicle in the left lane.
					&& (speed_of_car[L*(lane_token-1)+i]<0)//the nearest-neighbour site in the right lane is empty.
					&& (pos_diff_right_behind-1>=Vmax[0])//the maximum possible speed of the vehicle behind in the right lane is smaller than the gap in between.
					&& (type_of_car[L*lane_token+i]==1 ||//a fast vehicle does not shift from the left to the right lane if it finds a vehicle of "slow" type in front in the right lane within a distance of Vfmax.
						(type_of_car[L*lane_token+i]==0 && !(type_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]==1 && Vmax[0]>pos_diff_right_front-1))
						)){
							if((double) rand() / (RAND_MAX+1.0) < plc)//A vehicle changes lane with probability plc provided the conditions above.
								position_of_lane[L*lane_token+i]=2;
				}
			}
			else if(type_of_car[L*lane_token+i]==0){//Sy. for fast type
				int left_token=0, right_token=0;
				//right->left
				if(lane_token+1<num_of_lanes
				&& std::min(speed_of_car[L*lane_token+i]+1, Vmax[type_of_car[L*lane_token+i]])>pos_diff_current_front-1//the gap in front in the current lane is less than v + 1.
				&& pos_diff_left_front>pos_diff_current_front//the gap in front in the other lane is larger than the gap available in front in the current lane.
				&& speed_of_car[L*(lane_token+1)+i]<0//the nearest-neighbour site in the other lane is empty.
				&& pos_diff_left_behind-1>=Vmax[0]){//the maximum possible speed of the vehicle behind in the other lane is smaller than the gap in between.
					//A vehicle changes lane with probability plc provided the conditions above.
					left_token=1;
				}
				//left->right
				if(lane_token-1>=0
				&& std::min(speed_of_car[L*lane_token+i]+1, Vmax[type_of_car[L*lane_token+i]])>pos_diff_current_front-1//the gap in front in the current lane is less than v + 1.
				&& pos_diff_right_front>pos_diff_current_front//the gap in front in the other lane is larger than the gap available in front in the current lane.
				&& speed_of_car[L*(lane_token-1)+i]<0//the nearest-neighbour site in the other lane is empty.
				&& pos_diff_right_behind-1>=Vmax[0]){//the maximum possible speed of the vehicle behind in the other lane is smaller than the gap in between.
					//A vehicle changes lane with probability plc provided the conditions above.
					right_token=1;
				}
				
				if(left_token==1 && right_token==1){
					if(pos_diff_left_front>pos_diff_right_front){
						if((double) rand() / (RAND_MAX+1.0) < plc)
							position_of_lane[L*lane_token+i]=1;
					}
					else if(pos_diff_left_front<pos_diff_right_front){
						if((double) rand() / (RAND_MAX+1.0) < plc)
							position_of_lane[L*lane_token+i]=2;
					}
					else if(pos_diff_left_front==pos_diff_right_front){
						if((double) rand() / (RAND_MAX+1.0) < 0.5){
							if((double) rand() / (RAND_MAX+1.0) < plc)
								position_of_lane[L*lane_token+i]=1;
						}else{
							if((double) rand() / (RAND_MAX+1.0) < plc)
								position_of_lane[L*lane_token+i]=2;
						}
					}
				}else if(left_token==1){
					if((double) rand() / (RAND_MAX+1.0) < plc)
						position_of_lane[L*lane_token+i]=1;
				}else if(right_token==1){
					if((double) rand() / (RAND_MAX+1.0) < plc)
						position_of_lane[L*lane_token+i]=2;
				}
			}
			
		}	
	}
	for(int lane_token=0; lane_token<num_of_lanes-2; lane_token++){//To prevent collision.
		for(int i = 0; i < L; i++){
			if(position_of_lane[L*lane_token+i]==1 && position_of_lane[L*(lane_token+2)+i]==2){
				position_of_lane[L*lane_token+i]=0;
				position_of_lane[L*(lane_token+2)+i]=0;
			}
		}
	}
	
	
	for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
		for(int i=0; i<L; i++){
			if(lane_token+1<num_of_lanes && position_of_lane[L*lane_token+i]==1){//right->left
				std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token+1)+i]);
				std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token+1)+i]);
			}
			else if(lane_token-1>=0 && position_of_lane[L*lane_token+i]==2){//left->right
				std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token-1)+i]);
				std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token-1)+i]);
			}
		}
	}
	
}


void correlation(double *G, int *speed_of_car, int correlation_length, int L, int lane_token){
	
	int N_of_each_lane_of_t=0;
	
	for(int i=0; i<L; i++)
		if(speed_of_car[lane_token*L+i]>=0)
			N_of_each_lane_of_t++;
	
	for(int r=0, NiNir=0; r<correlation_length; r++){//r: pos_diff
		for(int i=0; i<L; i++)
			if(speed_of_car[lane_token*L+i]>-1 && speed_of_car[lane_token*L+(i+r)%L]>-1)
				NiNir++;
			else
				continue;
			
		G[lane_token*correlation_length+r]+=(double)NiNir/L-(double) (N_of_each_lane_of_t*N_of_each_lane_of_t)/(L*L);
		
		NiNir=0;
	}
	return;
}

void profile(double *Rho, int *speed_of_car, int L, int lane_token){
        //int NiNir=0;
        for(int i=0; i<L; i++){
                if(speed_of_car[lane_token*L+i]>-1)
        //                NiNir++;
        //        else
        //                continue;
		Rho[lane_token*L+i]++;
	//	Rho[lane_token*L+i]=Rho[lane_token*L+i]/L;	
        }
        return;	
}

int order_parameter(int *speed_of_car, int L, int lane_token){
	int NiNir=0;
	for(int i=0; i<L; i++){
		if(speed_of_car[lane_token*L+i]>-1 && speed_of_car[lane_token*L+(i+1)%L]>-1)
			NiNir++;
		else
			continue;
		}
	return NiNir;
}

double relaxation_time(double *Vavg_of_time_lane, int steps, double p, double V_steady_state){
	double tau=0;
	for(int t=0; t<steps+1; t++)
		tau+=std::min((1-p)*t, V_steady_state)-Vavg_of_time_lane[t];
	return tau;
}

int number_of_undertakings(int *speed_of_car, int L, int num_of_lanes){

	int num_of_undertakings=0;
	
	
	
	for(int lane_token=1; lane_token<num_of_lanes; lane_token++){
		for(int i = 0; i < L; i++){
			if(speed_of_car[L*lane_token+i]<=-1)
				continue;
			
			int pos_diff_right_front=1;
			if(lane_token-1>=0){
				while(speed_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]<0 && pos_diff_right_front<L){
					pos_diff_right_front++;
				}
			}
			if(speed_of_car[L*lane_token+i]+1+pos_diff_right_front-1<speed_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]){
				num_of_undertakings++;
			}
			
		}	
	}
	return num_of_undertakings;
	
}

int number_of_overtakings(int *speed_of_car, int L, int num_of_lanes){

	int num_of_overtakings=0;
	
	for(int lane_token=num_of_lanes-2; lane_token>=0; lane_token--){
		for(int i = 0; i < L; i++){
			if(speed_of_car[L*lane_token+i]<=-1)
				continue;
			
			
			int pos_diff_left_front=1;
			if(lane_token+1>=0){
				while(speed_of_car[L*(lane_token+1)+(i+pos_diff_left_front)%L]<0 && pos_diff_left_front<L){
					pos_diff_left_front++;
				}
			}
			
			if(speed_of_car[L*lane_token+i]+1+pos_diff_left_front-1<speed_of_car[L*(lane_token+1)+(i+pos_diff_left_front)%L]){
				num_of_overtakings++;
			}
			
		}	
	}
	return num_of_overtakings;
	
}

int main(int argc, char* argv[]){
	
	int L=1024, N=1, num_of_lanes=2, types=2, percentage=0, const_of_steps=10000, first_step_of_average=10*L, steps_of_average=5, sampling=10, steps=2*first_step_of_average, num_of_samples=100, digits_print=40, precision=20;
	int maxV=5;
	
	//Vmax[types]={5, 3};
	int *Vmax = new int[types];
	for(int i=0; i<types; i++){
		Vmax[i]=maxV-2*i;
	}
	
	
	int correlation_length=50/*(L-1)/2*/;
	int number_of_average_samples=0;
	int open_system=0;
	int leave_range=0, entering_speed=1;
	double p_enter=0.0;
	double Vavg=0, density=0, p=0.0, plc=1.0, p0=1.0;
	double fast_ratio=(double) 1;
	double cal_range=0.1;
	double designated_density_start=0, designated_density_end=0;
	
	
	char tmp, pch;
	std::stringstream ss, scheme_name, parameters, parameters_name_with_parts, parameters_name_without_parts;
	std::string sub_str;
	int sch=1, lane_ini_distribution_token=1;
	scheme_name.str("asy");
	void (*scheme_func_ptr)(int*, int*, int*, int, int, int*, double);
	scheme_func_ptr=asymmetric;
	void (*scheme_func_stage_ptr)(int*, int*, int*, int , int , int*, double, int&, int&, int&, int&, int, int, int, int, int);
	
#ifdef MULTI
	int iproc, nproc, ista, iend;
	
	MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &nproc);
	MPI_Comm_rank(MPI_COMM_WORLD, &iproc);
	std::srand(std::time(0)+iproc);// use current time as seed for random generator
#endif

#ifdef CPP11
#ifdef MULTI
	/* #include <random> */
	unsigned seed=time(NULL)+iproc;
	mt_rand.seed (seed);
#else
        unsigned seed=time(NULL);
        mt_rand.seed (seed);	
#endif
#endif

	//NS_open(speed_of_car+L*i, type_of_car+L*i, L, Vmax, p, leave_range, p_enter, entering_speed, fast_ratio);
#ifdef MULTI
	std::srand(std::time(0)+iproc);// use current time as seed for random generator
#else
	std::srand(std::time(0));// use current time as seed for random generator
#endif
	while((tmp=getopt(argc,argv,"hL:N:p:z:c:f:s:V:U:t:l:n:S:I:o"))!=-1){
		switch(tmp){
		
		case 'h':
			showhelpinfo(argv[0]);
			return 1;

		case 'L':		// length of the road
			L=atoi(optarg);
			break;

		case 'N':
			N=atoi(optarg);
			break;
			
		case 'p':		// randomization parameter
			ss.str("");
			ss.clear();
			p=atof(optarg);
			ss << optarg;
			getline(ss, sub_str,'/');
			if(getline(ss, sub_str,'/'))
				p/=atof(sub_str.c_str());
			break;

		case 'z':		// randomization parameter  p0 (VDR case)
                        ss.str("");
                        ss.clear();
                        p0=atof(optarg);
                        ss << optarg;
                        getline(ss, sub_str,'/');
                        if(getline(ss, sub_str,'/'))
                                        p0/=atof(sub_str.c_str());
                        break;
		
		case 'c':		// lane-changing probability: default: plc=1 
			ss.str("");
			ss.clear();
			plc=atof(optarg);
			ss << optarg;
			getline(ss, sub_str,'/');
			if(getline(ss, sub_str,'/'))
				plc/=atof(sub_str.c_str());
			break;
		
		
		case 'f':		// ratio of fast vehicles
			ss.str("");
			ss.clear();
			fast_ratio=atof(optarg);
			ss << optarg;
			getline(ss, sub_str,'/');
			if(getline(ss, sub_str,'/'))
				fast_ratio/=atof(sub_str.c_str());
			break;

		case 's':		// time steps (MC steps)
			steps=atoi(optarg);
			break;

		case 'V':		// maximum speed of the fast vehicles; default 5
			Vmax[0]=atoi(optarg);
			break;
		
		case 'U':		// maximum speed of the slow vehicles; default 3
			Vmax[1]=atoi(optarg);
			break;
		
		case 't':		// number of samples
			num_of_samples=atoi(optarg);
			break;
			
		case 'l':		// number of lanes
			num_of_lanes=atoi(optarg);
			break;
			
		case 'n':
			scheme_name.str("");
			scheme_name.clear();
			scheme_name.str(optarg);
			break;
			
		case 'S':		// scheme
			sch=atoi(optarg);
			break;
			
		case 'I':
			lane_ini_distribution_token=atoi(optarg);
			break;
			
		case 'o':
			OPEN=1;
			break;
		
		default:
			showhelpinfo(argv[0]);
			return 1;
		}
	}
	//input variables to be added.
	//for lane i.
	//P p, p0 
	//T lane_type[i]=?
	//o lane_type_open[i]=?
	//  lane_p[i]
	//A lane_p_enter[i]
	//  lane_entering_speed[i]
	//  lane_leave_range[i]
	//B x
	//D N_start, N_end.//for(N=N_start; N<N_end; N+=detN)//N: number of vehicles.
	//N_new, N_new_ensemble: averaged car number.
	//./a.out -L 1024 -s 100000 -p 0.5 -z 0.5 -f 0.75 -l 3 -S 1 -t 1 &
	//./a.out -L 1024 -s 100000 -p 0.5 -z 0.5 -f 0.75 -l 3 -S 8 -t 1 &
	
	if(L<=0 || N<=0 || p>=1 || p<0 || plc>1 || plc<0 || fast_ratio>1 || fast_ratio<0 || Vmax[0]<=0 || Vmax[1]<=0 || num_of_samples<1 || num_of_lanes<1 || sch>6 || lane_ini_distribution_token<1 || lane_ini_distribution_token>3){
		if(L<=0)
			std::cout << "L should be > 0." << std::endl;
			
		if(N<=0)
			std::cout << "N should be > 0." << std::endl;
		
		if(p>1 || p<0)
			std::cout << "p should be an element of [0,1). Input p value: " << p << "." << std::endl;
		
		if(plc>1 || plc<0)
			std::cout << "plc should be an element of [0,1). Input p value: " << plc << "." << std::endl;
		
		if(fast_ratio>1 || fast_ratio<0)
			std::cout << "fsr should be an element of [0,1). Input fsr value: " << fast_ratio << "." << std::endl;
		
		if(Vmax[0]<=0 || Vmax[1]<=0)
			std::cout << "Vmax should be > 0." << std::endl;
			
		if(num_of_samples<1)
			std::cout << "num of times should be > 0." << std::endl;
		
		if(num_of_lanes<1)
			std::cout << "num of lanes should be > 0." << std::endl;
		
		if(sch>6)
			std::cout << "S should be [1, 6]." << std::endl;
		
		if(lane_ini_distribution_token<1 || lane_ini_distribution_token>3)
			std::cout << "I should be [1, 3]." << std::endl;
		
		return 1;
	}

	first_step_of_average=10*L;		// initial steps for "relaxation"
	if(steps<first_step_of_average+const_of_steps)
		steps=first_step_of_average+const_of_steps;
	steps=steps+first_step_of_average;


#ifdef PRINT_PARAMETERS
	std::cout 
	<< "L=" << L << std::endl
	<< "p=" << p << std::endl
	<< "s=" << steps << std::endl
	<< "Vmax=" << Vmax[0] << std::endl;
#endif
	
	
	double *flow = new double[L*num_of_lanes]();
	double *sflow = new double[L*num_of_lanes]();
	#ifdef COR_G
	double *G = new double[num_of_lanes*correlation_length]();
	#endif
        #ifdef PROFILE
        double *Rho = new double[num_of_lanes*L]();
        #endif
	
	int *position_of_lane = new int[L*num_of_lanes];
	int *speed_of_car = new int[L*num_of_lanes];
	int *type_of_car = new int[L*num_of_lanes];
	
	double *Vavg_of_time_of_each_lane = new double[(num_of_lanes)*(steps+1)];
	int *number_of_vehicle_of_time_of_each_lane = new int[(num_of_lanes)*(steps+1)];
	
	
	double *Vavg_of_time_of_all_lanes = new double[steps+1];
	double Vavg_over_time_of_all_lanes = 0;
	
	double *Flow_over_time_of_each_lane = new double[num_of_lanes];
	double *Vavg_over_time_of_each_lane = new double[num_of_lanes];
	double *d_over_time_of_each_lane = new double[num_of_lanes];
	int *num_of_veh_of_lane = new int[num_of_lanes];
	int *num_of_fast_veh_of_lane = new int[num_of_lanes];
	
	int *lane_type = new int[num_of_lanes]();
	int *lane_type_open = new int[num_of_lanes]();
	double *lane_p = new double[num_of_lanes];
	double *lane_p_enter = new double[num_of_lanes];
	double *lane_p_leave = new double[num_of_lanes];
	int *lane_entering_speed = new int[num_of_lanes]();
	int *lane_leave_range = new int[num_of_lanes]();
	////////////  VDR ///////////////
	double *p_braking = new double[maxV+1]; 	
	
	
		
	double *Vavg_of_time_of_each_VehType = new double[types*(steps+1)];
	double *Vavg_over_time_of_each_VehType = new double[types];
	
	int N_new=0, flow_det=0, flow_det_exact_ratio=0, average_counter=0, det_location=10;
	int lane_changing_number=0;
	int num_of_undertakings=0;
	int num_of_overtakings=0;
		
	int *flow_lane_det = new int[num_of_lanes]();
	int *flow_lane_det_exact_ratio = new int[num_of_lanes]();
	int *lane_usage = new int[num_of_lanes]();
	int *m = new int[num_of_lanes]();
	int *lane_usage_type = new int[types*num_of_lanes]();
	int *num_of_times_of_lane = new int[num_of_lanes]();
	int *Vavg_token_of_lane = new int[num_of_lanes]();
	
	int *flow_type_det = new int[types]();
	int *flow_type_det_exact_ratio = new int[types]();
	
	/* Ensemble averages */
	double N_new_ensemble=0, fout_ensemble=0, fdetout_ensemble=0, fdetexactout_ensemble=0, Vout_ensemble=0, lane_changing_number_ensemble=0, num_of_undertakings_ensemble=0, num_of_overtakings_ensemble=0, tau_ensemble=0;
	double *mout_ensemble = new double[num_of_lanes]();
	double *mdetout_ensemble = new double[num_of_lanes]();
	double *mdetexactout_ensemble = new double[num_of_lanes]();
	double *mVout_ensemble = new double[num_of_lanes]();
	double *mdout_ensemble = new double[num_of_lanes]();
	double *luout_ensemble = new double[num_of_lanes]();
	double *m_ensemble = new double[num_of_lanes]();
	double *lane_usage_type_ensemble = new double[types*num_of_lanes]();
	double *type_out_ensemble = new double[types]();
	double *type_det_out_ensemble = new double[types]();
	double *type_det_exact_out_ensemble = new double[types]();
	double *type_Vout_ensemble = new double[types]();
	#ifdef COR_G
	double *G_ensemble = new double[num_of_lanes*correlation_length]();
	#endif
        #ifdef PROFILE
        double *Rho_ensemble = new double[num_of_lanes*L]();
        #endif
	double *Vavg_of_time_of_all_lanes_ensemble = new double[steps+1]();
	
	for(int i=0; i<num_of_lanes;i++){
		lane_p[i]=p;
		lane_leave_range[i]=leave_range;
		lane_p_enter[i]=p_enter;
		lane_entering_speed[i]=entering_speed;			
	}

	/* Models */
        //input variables to be added.
        //for lane i.
        //P lane_p[i]=p,p0
        //T lane_type[i]=?   (0: driving lane; 1: opvertaking lane)
        //o lane_type_open[i]=? (0: closed; 1: open)
        //A lane_p_enter[i]
        //  lane_entering_speed[i]=entering_speed (default: 1)
        //  lane_leave_range[i]=leave_range (default: 0)
        ////////////////////////////////////////// 
        //  sch==1  (-S 1): closed asymmetric model
        //  sch==2  (-S 2): closed symmetric model
        //  sch==3  (-S 3): closed hybrid model
        //  sch==4  (-S 4): open asymmetric model (one open lane)
        //  sch==5  (-S 5): open symmetric model  (one open lane) 
        //  sch==6  (-S 6): open hybrid model     (one open lane)

        //==== VDR ============ //
	for(int i=1; i<Vmax[0]; i++)
         {
           p_braking[i]=p;
         }
        p_braking[Vmax[0]]=0;  // cruise control limit
        p_braking[1]=p0;

        for(int i=1; i<Vmax[0]; i++) 
	 {
	   if(p_braking[i]!=p)	
	        VDR=1;
	 }
	//======================//
	
	scheme_name.str("");
	scheme_name.clear();
        if(num_of_lanes==1 && (!OPEN)){
                scheme_name << "sg";
                scheme_func_ptr=asymmetric;
        }
        else if(num_of_lanes==1 && OPEN){
                scheme_name << "sg_o";
                scheme_func_ptr=asymmetric;
                lane_type_open[0]=1;
        }
	else if(sch==1){//scheme i
		scheme_name << "asy";
		scheme_func_ptr=asymmetric;
		lane_type[0]=1;
		lane_p[0]=p;
		for(int i=1; i<num_of_lanes; i++){
			lane_type[i]=1;
			lane_p[i]=p;
			lane_type_open[i]=0;
		}
	}
	else if(sch==2){
		scheme_name << "sy";
		scheme_func_ptr=symmetric;
		for(int i=0; i<num_of_lanes; i++){
			lane_type[i]=0;
			lane_p[i]=p;
			lane_type_open[i]=0;
		}
	}
	else if(sch==3){
                scheme_name << "hybrid";
                scheme_func_ptr=hybrid;
                for(int i=0; i<num_of_lanes-1; i++){      
		        lane_type[i]=0;
		        lane_p[i]=p;	
		}	
                lane_type[num_of_lanes-1]=1;
		lane_p[num_of_lanes-1]=p;
                for(int i=0; i<num_of_lanes; i++)
                        lane_type_open[i]=0;
        }
	else if(sch==4){
		scheme_name << "asy_o";
		scheme_func_ptr=asymmetric;
		lane_type[0]=0;
		lane_p[0]=p;
		lane_type[1]=0;
                lane_p[1]=p;
		for(int i=2; i<num_of_lanes; i++){
			lane_type[i]=1;
			lane_p[i]=p;
			lane_type_open[i]=0;
		}
		lane_type_open[0]=1;
		lane_ini_distribution_token=2;
	}
        else if(sch==5){
                scheme_name << "sy_o";
                scheme_func_ptr=symmetric;
                for(int i=0; i<num_of_lanes; i++){
                        lane_type[i]=0;
                        lane_p[i]=p;
                        lane_type_open[i]=0;
                }
                lane_type_open[0]=1;
                lane_ini_distribution_token=2;		
        }	
        else if(sch==6){
                scheme_name << "hybrid_o";
                scheme_func_ptr=hybrid;
                for(int i=0; i<num_of_lanes-1; i++){
                        lane_type[i]=0;
                        lane_p[i]=p;
                        lane_type_open[i]=0;
                }
                lane_type[num_of_lanes-1]=1;
                lane_p[num_of_lanes-1]=p;
                lane_type_open[0]=1;
                lane_ini_distribution_token=2;
        }
	if(lane_ini_distribution_token==2){//lane inital condition name
		scheme_name << "_r" ;
	}
	else if(lane_ini_distribution_token==3){
		scheme_name << "_s1" ;
	}

	for(int i=0; i<num_of_lanes; i++)
         {
	   if(lane_type_open[i]) OPEN=1;
         }

	//==  Output files ==== //

        if(VDR)
        parameters << "_L" << L << "_nl" << num_of_lanes << "_p" << p << "_zp" << p0 << "_plc" << plc << "_f" << fast_ratio << "_V" << Vmax[0] << "U" << Vmax[1];
        else
        parameters << "_L" << L << "_nl" << num_of_lanes << "_p" << p << "_plc" << plc << "_f" << fast_ratio << "_V" << Vmax[0] << "U" << Vmax[1];
        #ifdef MULTI
        parameters_name_with_parts << parameters.str() << "_part" << std::setfill('0') << std::setw(2) << iproc << ".dat";
        #else
        parameters_name_with_parts << parameters.str() << ".dat";
        #endif
        parameters_name_without_parts << parameters.str() << ".dat";

	
	//fd file name
	ss.str("");
	ss.clear();
	
	ss << "fd" 
	<< "_total" << "_" << scheme_name.str() << parameters_name_with_parts.str();
	std::ofstream fout(ss.str().c_str());
	if(!fout){
		std::cout << "Error for file writing." << std::endl;
		return 1;
	}
	fout << std::fixed << std::setprecision(precision);
			
	//fd_discrete file name
	ss.str("");
	ss.clear();
	
	ss << "fd_discrete" 
	<< "_total" << "_" << scheme_name.str() << parameters_name_with_parts.str();
	std::ofstream fd_discrete_out(ss.str().c_str());
	if(!fd_discrete_out){
		std::cout << "Error for file writing." << std::endl;
		return 1;
	}
	fd_discrete_out << std::fixed << std::setprecision(precision);
	
	//fd file name
	ss.str("");
	ss.clear();
	
	
#ifdef EX	
	//fd exact ratio file name
	ss.str("");
	ss.clear();
	
	ss << "fdex" 
	<< "_total" << "_" << scheme_name.str() << parameters_name_with_parts.str();
	std::ofstream fexout(ss.str().c_str());
	if(!fexout){
		std::cout << "Error for file writing." << std::endl;
		return 1;
	}
	fexout << std::fixed << std::setprecision(precision);
	
#endif
	//fd det file name
	ss.str("");
	ss.clear();
	
	ss << "fdet" 
	<< "_total" << "_" << scheme_name.str() << parameters_name_with_parts.str();
	std::ofstream fdetout(ss.str().c_str());
	if(!fdetout){
		std::cout << "Error for file writing." << std::endl;
		return 1;
	}
	fdetout << std::fixed << std::setprecision(precision);

	//V file name (average velocity)
	ss.str("");
	ss.clear();
	
	ss << "V" 
	<< "_total" << "_" << scheme_name.str() << parameters_name_with_parts.str();
	std::ofstream Vout(ss.str().c_str());
	if(!Vout){
		std::cout << "Error for file writing." << std::endl;
		return 1;
	}
	Vout << std::fixed << std::setprecision(precision);
	

	//fd of lane i file name  (fundamental diagrams (flow-density) of lane i)
	std::ofstream *mout= new std::ofstream[num_of_lanes];
	for(int i=0; i<num_of_lanes; i++){
		
		ss.str("");
		ss.clear();
		
		ss << "fd" 
		<< "_lane" << i << "_" << scheme_name.str() << parameters_name_with_parts.str();
		(mout+i)->open(ss.str().c_str());
		if(!(mout+i)->is_open()){
			std::cout << "Error for file writing." << std::endl;
			return 1;
		}
		mout[i] << std::fixed << std::setprecision(precision);
		
	}

	
	//fd det of lane i file name
	std::ofstream *mdetout= new std::ofstream[num_of_lanes];
	for(int i=0; i<num_of_lanes; i++){
		
		ss.str("");
		ss.clear();
		
		ss << "fdet" 
		<< "_lane" << i << "_" << scheme_name.str() << parameters_name_with_parts.str();
		(mdetout+i)->open(ss.str().c_str());
		if(!(mdetout+i)->is_open()){
			std::cout << "Error for file writing." << std::endl;
			return 1;
		}
		mdetout[i] << std::fixed << std::setprecision(precision);
		
		
	}

	//V of lane i file name	(average velocity of lane i)
	std::ofstream *mVout= new std::ofstream[num_of_lanes];
	for(int i=0; i<num_of_lanes; i++){
		
		ss.str("");
		ss.clear();
		
		ss << "V" 
		<< "_lane" << i << "_" << scheme_name.str() << parameters_name_with_parts.str();
		(mVout+i)->open(ss.str().c_str());
		if(!(mVout+i)->is_open()){
			std::cout << "Error for file writing." << std::endl;
			return 1;
		}
		mVout[i] << std::fixed << std::setprecision(precision);
		
		
	}

	//density of lane i file name  (density of lane i)
	std::ofstream *mdout= new std::ofstream[num_of_lanes];
	for(int i=0; i<num_of_lanes; i++){
		
		ss.str("");
		ss.clear();
		
		ss << "d" 
		<< "_lane" << i << "_" << scheme_name.str() << parameters_name_with_parts.str();
		(mdout+i)->open(ss.str().c_str());
		if(!(mdout+i)->is_open()){
			std::cout << "Error for file writing." << std::endl;
			return 1;
		}
		mdout[i] << std::fixed << std::setprecision(precision);
		
		
	}

	//lane usage
	std::ofstream *luout= new std::ofstream[num_of_lanes];
	for(int i=0; i<num_of_lanes; i++){
		
		ss.str("");
		ss.clear();
		
		ss << "lu" 
		<< "_lane" << i << "_" << scheme_name.str() << parameters_name_with_parts.str();
		(luout+i)->open(ss.str().c_str());
		if(!(luout+i)->is_open()){
			std::cout << "Error for file writing." << std::endl;
			return 1;
		}
		luout[i] << std::fixed << std::setprecision(precision);
		
		
	}

	//lane usages of types (fast=type0 or slow=type1)
	std::ofstream *lutypeout= new std::ofstream[types*num_of_lanes];
	for(int i=0; i<types; i++){
		for(int j=0; j<num_of_lanes; j++){
			
			ss.str("");
			ss.clear();
			
			ss << "lu" 
			<< "_type" << i << "_lane" << j << "_" << scheme_name.str() << parameters_name_with_parts.str();
			(lutypeout+i*num_of_lanes+j)->open(ss.str().c_str());
			if(!(lutypeout+i*num_of_lanes+j)->is_open()){
				std::cout << "Error for file writing." << std::endl;
				return 1;
			}
			lutypeout[i*num_of_lanes+j] << std::fixed << std::setprecision(precision);
			
		}
	}

	//fd of type i file name (flow-density relations of car types)
	std::ofstream *type_out= new std::ofstream[types];
	for(int i=0; i<types; i++){
		
		ss.str("");
		ss.clear();
		
		ss << "fd" 
		<< "_type" << i << "_" << scheme_name.str() << parameters_name_with_parts.str();
		(type_out+i)->open(ss.str().c_str());
		if(!(type_out+i)->is_open()){
			std::cout << "Error for file writing." << std::endl;
			return 1;
		}
		type_out[i] << std::fixed << std::setprecision(precision);
		
	}

	//fd det of type i file name
	std::ofstream *type_det_out= new std::ofstream[types];
	for(int i=0; i<types; i++){
		
		ss.str("");
		ss.clear();
		
		ss << "fdet" 
		<< "_type" << i << "_" << scheme_name.str() << parameters_name_with_parts.str();
		(type_det_out+i)->open(ss.str().c_str());
		if(!(type_det_out+i)->is_open()){
			std::cout << "Error for file writing." << std::endl;
			return 1;
		}
		type_det_out[i] << std::fixed << std::setprecision(precision);
		
	}

	//V of type i file name
	std::ofstream *type_Vout= new std::ofstream[types];
	for(int i=0; i<types; i++){
		
		ss.str("");
		ss.clear();
		
		ss << "V" 
		<< "_type" << i << "_" << scheme_name.str() << parameters_name_with_parts.str();
		(type_Vout+i)->open(ss.str().c_str());
		if(!(type_Vout+i)->is_open()){
			std::cout << "Error for file writing." << std::endl;
			return 1;
		}
		type_Vout[i] << std::fixed << std::setprecision(precision);
		
	}

	//tau file name
	ss.str("");
	ss.clear();
	
	ss << "tau" 
	<< "_total" << "_" << scheme_name.str() << parameters_name_with_parts.str();
	std::ofstream tau_file(ss.str().c_str());
	if(!tau_file){
		std::cout << "Error for file writing." << std::endl;
		return 1;
	}
	tau_file << std::fixed << std::setprecision(precision);
	

	//number of lane changing file name
	ss.str("");
	ss.clear();
	
	ss << "lcn" 
	<< "_total" << "_" << scheme_name.str() << parameters_name_with_parts.str();
	std::ofstream lcnout(ss.str().c_str());
	if(!lcnout){
		std::cout << "Error for file writing." << std::endl;
		return 1;
	}
	lcnout << std::fixed << std::setprecision(precision);
	

	//number of undertakings file name
	ss.str("");
	ss.clear();
	
	ss << "utn" 
	<< "_total" << "_" << scheme_name.str() << parameters_name_with_parts.str();
	std::ofstream utout(ss.str().c_str());
	if(!utout){
		std::cout << "Error for file writing." << std::endl;
		return 1;
	}
	utout << std::fixed << std::setprecision(precision);

	//number of overtakings file name
	ss.str("");
	ss.clear();
	
	ss << "otn" 
	<< "_total" << "_" << scheme_name.str() << parameters_name_with_parts.str();
	std::ofstream otout(ss.str().c_str());
	if(!otout){
		std::cout << "Error for file writing." << std::endl;
		return 1;
	}
	otout << std::fixed << std::setprecision(precision);

	//V_f file name
	ss.str("");
	ss.clear();
	
	ss << "V_f" 
	<< "_total" << "_" << scheme_name.str() << parameters_name_with_parts.str();
	std::ofstream V_f_out(ss.str().c_str());
	if(!V_f_out){
		std::cout << "Error for file writing." << std::endl;
		return 1;
	}
	V_f_out << std::fixed << std::setprecision(precision);
	

	
	//V_fdet file name
	ss.str("");
	ss.clear();
	
	ss << "V_fdet" 
	<< "_total" << "_" << scheme_name.str() << parameters_name_with_parts.str();
	std::ofstream V_fdet_out(ss.str().c_str());
	if(!V_fdet_out){
		std::cout << "Error for file writing." << std::endl;
		return 1;
	}
	V_fdet_out << std::fixed << std::setprecision(precision);
	

	// order parameter file name
	std::ofstream *order_out= new std::ofstream[num_of_lanes];
	for(int i=0; i<num_of_lanes; i++){
		
		ss.str("");
		ss.clear();
		
		ss << "order" 
		<< "_lane" << i << "_" << scheme_name.str() << parameters_name_with_parts.str();
		(order_out+i)->open(ss.str().c_str());
		if(!(order_out+i)->is_open()){
			std::cout << "Error for file writing." << std::endl;
			return 1;
		}
		order_out[i] << std::fixed << std::setprecision(precision);
		
	}

	//==== Output files (end) ===//	
	
	//==== Parameter distribution ===//    
	
#ifdef MULTI
	para_range(1, L*num_of_lanes, nproc, iproc, ista, iend);
#endif

	int detN=0;
	for(detN=1; detN<L*num_of_lanes; detN++){
		if((int) ceil((double) detN*fast_ratio)==(int) (detN*fast_ratio) && detN-(int) ceil((double) detN*fast_ratio)==detN-(int) (detN*fast_ratio))
			break;
	}
	//lane homogeneous
	detN*=num_of_lanes;
	int N_start=0, N_end=0;
	
	
	if(designated_density_start>0 && designated_density_end>0){
		N_start=designated_density_start*L*num_of_lanes;
		N_end=designated_density_end*L*num_of_lanes;
	}
	else{
		#ifdef MULTI
		N_start=detN+iproc*detN;
		#else
		N_start=detN;
		#endif
		N_end=L*num_of_lanes;
	}
	
	double E_start=0., E_end=1., detE=0.05;
	if(OPEN)
	{
		#ifdef MULTI
		if(1./nproc<detE) detE=0.25/nproc;
		E_start=detE+iproc*detE;
		#else
		E_start=detE;
		#endif	       	
		E_end=1+0.5*detE;
	}

// +++++++++++++++++++++++++++++++++++++++++++

	#ifdef MULTI
	 double counter_start, counter_end, counter_delta;
         if(!OPEN)
	  {
            counter_start=N_start;
 	    counter_end=N_end;
	    counter_delta=detN;
          }
         else
          {
            counter_start=E_start;
            counter_end=E_end;
            counter_delta=detE;
          } 	 
	  for(double ic=counter_start; ic<counter_end; ic+=nproc*counter_delta){
          if(!OPEN)
	      N=(int)ic;
	  else
	    {
              p_enter=ic;
	      for(int i=0; i<num_of_lanes;i++){
                lane_p_enter[i]=p_enter;
              }	
            }	
	#else
         double counter_start, counter_end, counter_delta; 
         if(!OPEN)
          {
            counter_start=N_start;
            counter_end=N_end;
            counter_delta=detN;
          }
         else
          {
            counter_start=E_start;
            counter_end=E_end;
            counter_delta=detE;
          }
	  for(double ic=counter_start; ic<counter_end; ic+=counter_delta){
          if(!OPEN)
              N=(int)ic;
          else
            {
              p_enter=ic;
              for(int i=0; i<num_of_lanes;i++){
                lane_p_enter[i]=p_enter;
              }
            }
	#endif
// -----------------------------------		
		#ifdef COR_G
		std::ofstream *Gout= new std::ofstream[num_of_lanes];
		for(int i=0; i<num_of_lanes; i++){
			
			ss.str("");
			ss.clear();

                if(OPEN){
                        ss << "G"
                        << "_enter" << p_enter << "_lane" << i << "_" << scheme_name.str() << parameters_name_without_parts.str();
                }
		else{			
			ss << "G" 
			<< "_d" << (double) N/(L*num_of_lanes) << "_lane" << i << "_" << scheme_name.str() << parameters_name_without_parts.str();
		}
			(Gout+i)->open(ss.str().c_str());
			if(!(Gout+i)->is_open()){
				std::cout << "Error for file writing." << std::endl;
				return 1;
			}
			
		}
		#endif
                #ifdef PROFILE
                std::ofstream *Rhoout= new std::ofstream[num_of_lanes];
                for(int i=0; i<num_of_lanes; i++){

                        ss.str("");
                        ss.clear();

                if(OPEN){
                        ss << "rho"
                        << "_enter" << p_enter << "_lane" << i << "_" << scheme_name.str() << parameters_name_without_parts.str();
                }
                else{
                        ss << "rho"
                        << "_d" << (double) N/(L*num_of_lanes) << "_lane" << i << "_" << scheme_name.str() << parameters_name_without_parts.str();
                }
                        (Rhoout+i)->open(ss.str().c_str());
                        if(!(Rhoout+i)->is_open()){
                                std::cout << "Error for file writing." << std::endl;
                                return 1;
                        }

                }
                #endif
	
		#ifdef TRACE
		int print_steps=1024;
		std::ofstream *trace= new std::ofstream[num_of_lanes];
		for(int i=0; i<num_of_lanes; i++){
			ss.str("");
			ss.clear();
		if(OPEN){
                        ss << "trace"
                        << "_enter" << p_enter << "_lane" << i << "_" << scheme_name.str() << parameters_name_without_parts.str();
		}
		else{	
			ss << "trace" 
			<< "_d" << (double) N/(L*num_of_lanes) << "_lane" << i << "_" << scheme_name.str() << parameters_name_without_parts.str();
		}
			(trace+i)->open(ss.str().c_str());
			if(!(trace+i)->is_open()){
				std::cout << "Error for file writing." << std::endl;
				return 1;
			}
			trace[i] << std::fixed << std::setprecision(precision);
			trace[i] << "P3" << std::endl;
			trace[i] << L << " " << print_steps << std::endl;
			trace[i] << "255" << std::endl;
		}
		#endif
		
		
		/* Ensemble averages set to 0 */
		N_new_ensemble=0; fout_ensemble=0; fdetout_ensemble=0; fdetexactout_ensemble=0; Vout_ensemble=0; lane_changing_number_ensemble=0; num_of_undertakings_ensemble=0; num_of_overtakings_ensemble=0; tau_ensemble=0;
		for(int i=0; i<num_of_lanes; i++){
			mout_ensemble[i]=0;
			mdetout_ensemble[i]=0;
			mdetexactout_ensemble[i]=0;
			mVout_ensemble[i]=0;
			mdout_ensemble[i]=0;
			luout_ensemble[i]=0;
			m_ensemble[i]=0;
		}
		for(int i=0; i<types; i++){
			type_out_ensemble[i]=0;
			type_det_out_ensemble[i]=0;
			type_det_exact_out_ensemble[i]=0;
			type_Vout_ensemble[i]=0;
			for(int j=0; j<num_of_lanes; j++){
				lane_usage_type_ensemble[i*num_of_lanes+j]=0;
			}
		}
		#ifdef COR_G
		for(int i=0; i<num_of_lanes*correlation_length; i++)
			G_ensemble[i]=0;
		#endif
                #ifdef PROFILE
                for(int i=0; i<num_of_lanes*L; i++)
                        Rho_ensemble[i]=0;
                #endif
		for(int i=0; i<steps+1; i++){
			Vavg_of_time_of_all_lanes_ensemble[i]=0;
		}
		
		if(lane_ini_distribution_token==3){//sg. only one slow vehicle. lane uniform distributed.
			//number of total veh.
			for(int i=0; i<num_of_lanes; i++){
				num_of_veh_of_lane[i]=0;
			}
			int Remained_n=N;
			
			for(int i=0; i<num_of_lanes; i++){
				if(Remained_n>N/num_of_lanes){
					num_of_veh_of_lane[i]=N/num_of_lanes;
					Remained_n-=N/num_of_lanes;
				}
				else{
					num_of_veh_of_lane[i]=Remained_n;
					break;
				}
				
			}
			//number of fast veh.
			for(int i=0; i<num_of_lanes; i++){
				num_of_fast_veh_of_lane[i]=0;
			}
			Remained_n=N-1;
			for(int i=0; i<num_of_lanes; i++){
				if(Remained_n>N-1/num_of_lanes){
					num_of_fast_veh_of_lane[i]=N-1/num_of_lanes;
					Remained_n-=N-1/num_of_lanes;
				}
				else{
					num_of_fast_veh_of_lane[i]=Remained_n;
					break;
				}
			}
		}
		else if(lane_ini_distribution_token==2){//right lane first distributed.
			//number of total veh.
			for(int i=0; i<num_of_lanes; i++){
				num_of_veh_of_lane[i]=0;
			}
			int Remained_n=N;
			
			for(int i=0; i<num_of_lanes; i++){
				if(Remained_n>L){
					num_of_veh_of_lane[i]=L;
					Remained_n-=L;
				}
				else if(Remained_n<=L){
					if(i<num_of_lanes-1 && Remained_n==L){
						num_of_veh_of_lane[i]=L-1;
						Remained_n-=L-1;
						continue;
					}
					else{
						num_of_veh_of_lane[i]=Remained_n;
						break;
					}
				}
			}
			//number of fast veh.
			for(int i=0; i<num_of_lanes; i++){
				num_of_fast_veh_of_lane[i]=0;
			}
			Remained_n=ceil((double) N*fast_ratio);
			for(int i=num_of_lanes-1; i>=0; i--){
				if(Remained_n>num_of_veh_of_lane[i]){
					num_of_fast_veh_of_lane[i]=num_of_veh_of_lane[i];
					Remained_n-=num_of_veh_of_lane[i];
				}
				else if(Remained_n<=num_of_veh_of_lane[i]){
					num_of_fast_veh_of_lane[i]=Remained_n;
					break;
				}
			}
		}
		else{//lane uniform distributed.
			//number of total veh.
			for(int i=0; i<num_of_lanes; i++){
				num_of_veh_of_lane[i]=0;
			}
			int Remained_n=N;
			
			for(int i=0; i<num_of_lanes; i++){
				if(i<N%num_of_lanes){//remainder
					num_of_veh_of_lane[i]=N/num_of_lanes+1;
				}
				else{
					num_of_veh_of_lane[i]=N/num_of_lanes;
					
				}
				
			}
			//number of fast veh.
			for(int i=0; i<num_of_lanes; i++){
				num_of_fast_veh_of_lane[i]=0;
			}
			int N_f=ceil((double) N*fast_ratio);
			for(int i=0; i<num_of_lanes; i++){
				if(i<N_f%num_of_lanes){//remainder
					num_of_fast_veh_of_lane[i]=N_f/num_of_lanes+1;
				}
				else{
					num_of_fast_veh_of_lane[i]=N_f/num_of_lanes;
					
				}
			}
		}
		for(int i=0; i<num_of_lanes; i++){
			num_of_times_of_lane[i]=0;
		}
		
		//------------------------------ SIMULATION -------------------------------------------------------------------------------------------
		for(int ith_time=0; ith_time<num_of_samples; ith_time++){	// Samples
			
			for(int i=0; i<num_of_lanes; i++){
				lane_initial(position_of_lane+L*i, speed_of_car+L*i, type_of_car+L*i, num_of_veh_of_lane[i], L, num_of_fast_veh_of_lane[i]);
			}
			
			
			Vavg_of_time_of_all_lanes[0]=averageLane(speed_of_car, L*num_of_lanes);
			for(int i=0; i<num_of_lanes; i++){
				Vavg_of_time_of_each_lane[i*(steps+1)]=averageLane(speed_of_car+L*i, L);
				number_of_vehicle_of_time_of_each_lane[i*(steps+1)]=numVehiclesLane(speed_of_car+L*i, L);
			}
			for(int i=0; i<types; i++)
				Vavg_of_time_of_each_VehType[i*(steps+1)]=averageVehType(speed_of_car, type_of_car, L*num_of_lanes, i);
			
			N_new=0;
			flow_det=0;
			flow_det_exact_ratio=0;
			average_counter=0;
			lane_changing_number=0;
			num_of_undertakings=0;
			num_of_overtakings=0;
			
			
			for(int i=0; i<num_of_lanes; i++){
				flow_lane_det[i]=0;
				flow_lane_det_exact_ratio[i]=0;
				lane_usage[i]=0;
				m[i]=0;
			}
			
			for(int i=0; i<types; i++){
				flow_type_det[i]=0;
				flow_type_det_exact_ratio[i]=0;
				for(int j=0; j<num_of_lanes; j++){
					lane_usage_type[i*num_of_lanes+j]=0;
				}
				
			}

			#ifdef COR_G
			for(int i=0; i<num_of_lanes; i++){
				for(int j=0; j<correlation_length; j++){
					G[i*correlation_length+j]=0;
				}
				
			}
			#endif
                        #ifdef PROFILE
                        for(int i=0; i<num_of_lanes; i++){
                                for(int j=0; j<L; j++){
                                        Rho[i*L+j]=0;
                                }

                        }
                        #endif
			
			//New process starts. (MC steps)
			for(int step=0; step<steps; step++){
				
				//scheme
				if(num_of_lanes>1){
					
					for(int i=0; i<num_of_lanes*L; i++)
						position_of_lane[i]=0;
					for(int lane_token=0; lane_token<num_of_lanes; lane_token++){//decision stage
						/*
						if(lane_type[lane_token]==1)
							scheme_func_stage_ptr=asymmetric_decision_stage;
						else if(lane_type[lane_token]==0)
							scheme_func_stage_ptr=symmetric_decision_stage;
						*/
						for(int i = 0; i < L; i++) {//for each veh. i on lane lane_token.
							
							if(speed_of_car[L*lane_token+i]<=-1)
								continue;
							
							int pos_diff_current_front=1;
							while(speed_of_car[L*lane_token+(i+pos_diff_current_front)%L]<0 && pos_diff_current_front<L){
								pos_diff_current_front++;
							}
							int pos_diff_right_front=1;
							if(lane_token-1>=0){
								while(speed_of_car[L*(lane_token-1)+(i+pos_diff_right_front)%L]<0 && pos_diff_right_front<L){
									pos_diff_right_front++;
								}
							}
							int pos_diff_right_behind=1;
							if(lane_token-1>=0){
								while(speed_of_car[L*(lane_token-1)+(i-pos_diff_right_behind+L)%L]<0 && pos_diff_right_behind<L){
									pos_diff_right_behind++;
								}
							}
							int pos_diff_left_front=1;
							if(lane_token+1<num_of_lanes){
								while(speed_of_car[L*(lane_token+1)+(i+pos_diff_left_front)%L]<0 && pos_diff_left_front<L){
									pos_diff_left_front++;
								}
							}
							int pos_diff_left_behind=1;
							if(lane_token+1<num_of_lanes){
								while(speed_of_car[L*(lane_token+1)+(i-pos_diff_left_behind+L)%L]<0 && pos_diff_left_behind<L){
									pos_diff_left_behind++;
								}
							}
							int left_token=0, right_token=0;
							if(lane_type[lane_token]==1)
								asymmetric_decision_stage(position_of_lane, speed_of_car, type_of_car, L, num_of_lanes, Vmax, plc, left_token, right_token, lane_token, i, pos_diff_current_front, pos_diff_right_front, pos_diff_right_behind, pos_diff_left_front, pos_diff_left_behind);
							else if(lane_type[lane_token]==0)
								symmetric_decision_stage(position_of_lane, speed_of_car, type_of_car, L, num_of_lanes, Vmax, plc, left_token, right_token, lane_token, i, pos_diff_current_front, pos_diff_right_front, pos_diff_right_behind, pos_diff_left_front, pos_diff_left_behind);
							
							
							
							if(left_token==1 && right_token==1){
								if(pos_diff_left_front>pos_diff_right_front){
									if((double) rand() / (RAND_MAX+1.0) < plc)
										position_of_lane[L*lane_token+i]=1;
								}
								else if(pos_diff_left_front<pos_diff_right_front){
									if((double) rand() / (RAND_MAX+1.0) < plc)
										position_of_lane[L*lane_token+i]=2;
								}
								else if(pos_diff_left_front==pos_diff_right_front){
									if((double) rand() / (RAND_MAX+1.0) < 0.5){
										if((double) rand() / (RAND_MAX+1.0) < plc)
											position_of_lane[L*lane_token+i]=1;
									}else{
										if((double) rand() / (RAND_MAX+1.0) < plc)
											position_of_lane[L*lane_token+i]=2;
									}
								}
							}else if(left_token==1){
								if((double) rand() / (RAND_MAX+1.0) < plc)
									position_of_lane[L*lane_token+i]=1;
							}else if(right_token==1){
								if((double) rand() / (RAND_MAX+1.0) < plc)
									position_of_lane[L*lane_token+i]=2;
							}
						}
					}
					
					for(int lane_token=0; lane_token<num_of_lanes-2; lane_token++){//To prevent collision.
						for(int i = 0; i < L; i++){
							if(position_of_lane[L*lane_token+i]==1 && position_of_lane[L*(lane_token+2)+i]==2){
								position_of_lane[L*lane_token+i]=0;
								position_of_lane[L*(lane_token+2)+i]=0;
							}
						}
					}


					for(int lane_token=0; lane_token<num_of_lanes; lane_token++){//change stage
						for(int i=0; i<L; i++){
							if(lane_token+1<num_of_lanes && position_of_lane[L*lane_token+i]==1){//right->left
								std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token+1)+i]);
								std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token+1)+i]);
							}
							else if(lane_token-1>=0 && position_of_lane[L*lane_token+i]==2){//left->right
								std::swap(speed_of_car[L*lane_token+i], speed_of_car[L*(lane_token-1)+i]);
								std::swap(type_of_car[L*lane_token+i], type_of_car[L*(lane_token-1)+i]);
							}
						}
					}
					
				}				
				for(int i=0; i<num_of_lanes; i++){
					#ifdef TRACE
					if(lane_type_open[i]==1){ //open system
						NS_open_part_1(speed_of_car+L*i, type_of_car+L*i, L, Vmax, lane_p[i], p_braking, lane_leave_range[i], lane_p_enter[i], lane_entering_speed[i], fast_ratio);
						
						if(ith_time==0 && steps-step<=print_steps){
							for(int j=0; j<L; j++){
								trace[i] << speed_of_car[L*i+j] << " ";
							}
							trace[i] << std::endl;
						
						}
						NS_open_part_2(speed_of_car+L*i, type_of_car+L*i, L, Vmax, lane_p[i], p_braking, lane_leave_range[i], lane_p_enter[i], lane_entering_speed[i], fast_ratio);
						
					}
					else{
						NS_part_1(speed_of_car+L*i, type_of_car+L*i, L, Vmax, lane_p[i], p_braking);
						if(ith_time==0 && steps-step<=print_steps){
							for(int j=0; j<L; j++){
								trace[i] << speed_of_car[L*i+j] << " ";
							}
							trace[i] << std::endl;
						
						}
						NS_part_2(speed_of_car+L*i, type_of_car+L*i, L, Vmax, lane_p[i], p_braking);
						
					}
					
					#else
					if(lane_type_open[i]==1){ //open system
						NS_open(speed_of_car+L*i, type_of_car+L*i, L, Vmax, lane_p[i], p_braking, lane_leave_range[i], lane_p_enter[i], lane_entering_speed[i], fast_ratio);
						
					}
					else{
						NS(speed_of_car+L*i, type_of_car+L*i, L, Vmax, lane_p[i], p_braking);
						
					}
					#endif
				}
				
				
				
				if(step>=first_step_of_average){
					average_counter++;
					N_new+=numVehiclesLane(speed_of_car, L*num_of_lanes);
					int N_temp=numVehiclesLane(speed_of_car, L*num_of_lanes);
					for(int lane_index=0; lane_index<num_of_lanes; lane_index++){
						for(int i=1; i<=Vmax[0]; i++){
							if(speed_of_car[L*lane_index+det_location+i]>=i){
								flow_det++;
								flow_lane_det[lane_index]++;
								
								for(int j=0; j<types; j++)
									if(type_of_car[L*lane_index+det_location+i]==j)
										flow_type_det[j]++;
								//exact ratio
								#ifdef EX
								if((int) ceil((double) N_temp*fast_ratio)==(int) (N_temp*fast_ratio) && N_temp-(int) ceil((double) N_temp*fast_ratio)==N_temp-(int) (N_temp*fast_ratio)){
									flow_det_exact_ratio++;
									flow_lane_det_exact_ratio[lane_index]++;
									
									for(int j=0; j<types; j++)
										if(type_of_car[L*lane_index+det_location+i]==j)
											flow_type_det_exact_ratio[j]++;
								
								}
								#endif
								break;
							}
						}
					}
					
					
					for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
						#ifdef COR_G
						//Computing correlation function G(r).
						if((double) N_temp/L >= (double) 1/(Vmax[0]+p+1)-cal_range || (double) N_temp/L <= (double) 1/(Vmax[0]+p+1)+cal_range){
							correlation(G, speed_of_car, correlation_length, L, lane_token);
						}
						//End of computing G(r) correlation function.
						#endif

                                                #ifdef PROFILE
						profile(Rho, speed_of_car, L, lane_token);	
                                                #endif
						
						lane_usage[lane_token]+=numVehiclesLane(speed_of_car+L*lane_token, L);
						m[lane_token]+=order_parameter(speed_of_car, L, lane_token);
						
					}
					//average_N+=numVehiclesLane(speed_of_car, L*num_of_lanes);
					
					for(int i=0; i<types; i++){
						for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
							lane_usage_type[i*num_of_lanes+lane_token]=numVehType(speed_of_car+L*lane_token, type_of_car+L*lane_token, L, i);
						}
					}
					
					for(int i=0; i<num_of_lanes*L; i++)
						if(position_of_lane[i]>0)
							lane_changing_number++;
						
					num_of_undertakings+=number_of_undertakings(speed_of_car, L, num_of_lanes);
					num_of_overtakings+=number_of_overtakings(speed_of_car, L, num_of_lanes);
					
					if((step-first_step_of_average)%((steps-first_step_of_average)/sampling)==0){
						/* total fd discrete */
						fd_discrete_out 
						<< std::setw(digits_print) << (double) numVehiclesLane(speed_of_car, L*num_of_lanes)/(L*num_of_lanes)
						<< std::setw(digits_print) << averageLane(speed_of_car, L*num_of_lanes)*numVehiclesLane(speed_of_car, L*num_of_lanes)/(L) << std::endl << std::flush;
					}
					
				}
				
				Vavg_of_time_of_all_lanes[step+1]=averageLane(speed_of_car, L*num_of_lanes);
				
				for(int i=0; i<num_of_lanes; i++){
					Vavg_of_time_of_each_lane[i*(steps+1)+(step+1)]=averageLane(speed_of_car+L*i, L);
					number_of_vehicle_of_time_of_each_lane[i*(steps+1)+(step+1)]=numVehiclesLane(speed_of_car+L*i, L);
				}
				for(int i=0; i<types; i++)
					Vavg_of_time_of_each_VehType[i*(steps+1)+(step+1)]=averageVehType(speed_of_car, type_of_car, L*num_of_lanes, i);
				
				
			}
			
			
			/*average over time of total and each-lane vehicles */
			number_of_average_samples=0;
			/* total */
			Vavg_over_time_of_all_lanes=0;
			for(int i=first_step_of_average; i<steps+1; i+=steps_of_average){
				Vavg_over_time_of_all_lanes+=Vavg_of_time_of_all_lanes[i];
				number_of_average_samples++;
			}
			Vavg_over_time_of_all_lanes/=number_of_average_samples;
			/* lanes */
			for(int j=0; j<num_of_lanes; j++){
				number_of_average_samples=0;
				Flow_over_time_of_each_lane[j]=0;
				for(int i=first_step_of_average; i<steps+1; i+=steps_of_average){
					Flow_over_time_of_each_lane[j]+=Vavg_of_time_of_each_lane[j*(steps+1)+i]*number_of_vehicle_of_time_of_each_lane[j*(steps+1)+i]/L;
					number_of_average_samples++;
				}
				Flow_over_time_of_each_lane[j]/=number_of_average_samples;
			}
			/* lanes V */
			for(int j=0; j<num_of_lanes; j++){
				number_of_average_samples=0;
				Vavg_over_time_of_each_lane[j]=0;
				Vavg_token_of_lane[j]=0;
				for(int i=first_step_of_average; i<steps+1; i+=steps_of_average){
					if(number_of_vehicle_of_time_of_each_lane[j*(steps+1)+i]>0){
						Vavg_over_time_of_each_lane[j]+=Vavg_of_time_of_each_lane[j*(steps+1)+i];
						number_of_average_samples++;
					}
				}
				if(number_of_average_samples!=0)
					Vavg_over_time_of_each_lane[j]/=number_of_average_samples;
				else
					Vavg_token_of_lane[j]=-1;//no vehicle in lane j.
				
			}
			/* lanes d */
			for(int j=0; j<num_of_lanes; j++){
				number_of_average_samples=0;
				d_over_time_of_each_lane[j]=0;
				for(int i=first_step_of_average; i<steps+1; i+=steps_of_average){
					d_over_time_of_each_lane[j]+=number_of_vehicle_of_time_of_each_lane[j*(steps+1)+i];
					number_of_average_samples++;
				}
				d_over_time_of_each_lane[j]/=number_of_average_samples*L;
			}
			
			/* VehType average over time. */
			for(int j=0; j<types; j++){
				number_of_average_samples=0;
				Vavg_over_time_of_each_VehType[j]=0;
				for(int i=first_step_of_average; i<steps+1; i+=steps_of_average){
					Vavg_over_time_of_each_VehType[j]+=Vavg_of_time_of_each_VehType[j*(steps+1)+i];
					number_of_average_samples++;
				}
				Vavg_over_time_of_each_VehType[j]/=number_of_average_samples;
			}	
			
			
			
			
			/* total fd */
			fout_ensemble+=Vavg_over_time_of_all_lanes*(N_new/average_counter)/L;
			/* total fd det */
			fdetout_ensemble+=(double) flow_det/average_counter;

			/* total V */
			Vout_ensemble+=Vavg_over_time_of_all_lanes;
			
			for(int i=0; i<num_of_lanes; i++){
				/* lanes fd */
				mout_ensemble[i]+=Flow_over_time_of_each_lane[i];
				/* lanes fd det */
				mdetout_ensemble[i]+=(double) flow_lane_det[i]/average_counter;
			}

			for(int i=0; i<num_of_lanes; i++){
				/* lanes V */
				if(Vavg_token_of_lane[i]==0){
					mVout_ensemble[i]+=Vavg_over_time_of_each_lane[i];
					num_of_times_of_lane[i]++;
				}
				/* lanes d */
				mdout_ensemble[i]+=d_over_time_of_each_lane[i];
				/* lane usage */
				luout_ensemble[i]+=(double) lane_usage[i]/((N_new/average_counter)*average_counter);
			}
			
			
			for(int i=0; i<types; i++){
				int type_num=0;
				for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
					type_num+=lane_usage_type[i*num_of_lanes+lane_token];
				}
				if(type_num!=0){
					for(int lane_token=0; lane_token<num_of_lanes; lane_token++){
						lane_usage_type_ensemble[i*num_of_lanes+lane_token]+=(double) lane_usage_type[i*num_of_lanes+lane_token]/type_num;	
					}
				}			
			}
			
			for(int i=0; i<types; i++){
				/* type fd */
				type_out_ensemble[i]+=Vavg_over_time_of_each_VehType[i] *numVehType(speed_of_car, type_of_car, L*num_of_lanes, i)/L;
				/* type fd det */
				type_det_out_ensemble[i]+=(double) flow_type_det[i]/average_counter;
			}

			/* types V */
			for(int i=0; i<types; i++){
				type_Vout_ensemble[i]+=Vavg_over_time_of_each_VehType[i];
			}
			#ifdef COR_G
			/* Spatial correlation */
			for(int i=0; i<num_of_lanes; i++){
				for(int j=0; j<correlation_length; j++){
					G_ensemble[i*correlation_length+j]+=G[i*correlation_length+j]/average_counter;
				}
				
			}
			#endif
                        #ifdef PROFILE
                        for(int i=0; i<num_of_lanes; i++){
                                for(int j=0; j<L; j++){
                                        Rho_ensemble[i*L+j]+=Rho[i*L+j]/average_counter;
                                }

                        }			
                        #endif
			
			for(int i=0; i<steps+1; i++){
				Vavg_of_time_of_all_lanes_ensemble[i]+=Vavg_of_time_of_all_lanes[i];
			}
			/* number of lane changing */
			lane_changing_number_ensemble+=(double) lane_changing_number/average_counter;
			
			/* number of undertakings */
			num_of_undertakings_ensemble+=(double) num_of_undertakings/average_counter;
			num_of_overtakings_ensemble+=(double) num_of_overtakings/average_counter;
			/* order_parameter */
			for(int i=0; i<num_of_lanes; i++){
				m_ensemble[i]+=(double) m[i]/average_counter;
			}
			
			N_new_ensemble+=(double) N_new/average_counter;
		}
		//----------------------------------------------------------------------------------------------------------------------------------------------------------
		
			N_new_ensemble/=num_of_samples;
			
			/* +++++++++ Output data +++++++++++ */
				
			
			/* total fd */
			if(!OPEN)
			fout 
			<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
			<< std::setw(digits_print) << fout_ensemble/num_of_samples << std::endl << std::flush;
			else
                        fout
                        << std::setw(digits_print) << p_enter
                        << std::setw(digits_print) << fout_ensemble/num_of_samples << std::endl << std::flush;
			
			
			/* total fd det */
			if(!OPEN)
			fdetout 
			<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
			<< std::setw(digits_print) << fdetout_ensemble/num_of_samples << std::endl << std::flush;
			else
			fdetout
                        << std::setw(digits_print) << p_enter 
                        << std::setw(digits_print) << fdetout_ensemble/num_of_samples << std::endl << std::flush;

			/* total V */
			if(!OPEN)
			Vout 
			<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
			<< std::setw(digits_print) << Vout_ensemble/num_of_samples << std::endl << std::flush;
			else
                        Vout
                        << std::setw(digits_print) << p_enter
                        << std::setw(digits_print) << Vout_ensemble/num_of_samples << std::endl << std::flush;

			/* V-F */
			if(!OPEN)
			V_f_out
			<< std::setw(digits_print) << fout_ensemble/num_of_samples
			<< std::setw(digits_print) << Vout_ensemble/num_of_samples << std::endl << std::flush;
			else
			V_f_out
                        << std::setw(digits_print) << p_enter
                        << std::setw(digits_print) << Vout_ensemble/num_of_samples << std::endl << std::flush;

			/* V-Fdet */
			if(!OPEN)
			V_fdet_out
			<< std::setw(digits_print) << fdetout_ensemble/num_of_samples
			<< std::setw(digits_print) << Vout_ensemble/num_of_samples << std::endl << std::flush;
			else
			V_fdet_out
                        << std::setw(digits_print) << p_enter
                        << std::setw(digits_print) << Vout_ensemble/num_of_samples << std::endl << std::flush;

			/* lanes fd */
			if(!OPEN)
			for(int i=0; i<num_of_lanes; i++){
				mout[i]
				<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
				<< std::setw(digits_print) << mout_ensemble[i]/num_of_samples << std::endl << std::flush;
			}
			else
                        for(int i=0; i<num_of_lanes; i++){
                                mout[i]
                                << std::setw(digits_print) << p_enter
                                << std::setw(digits_print) << mout_ensemble[i]/num_of_samples << std::endl << std::flush;
                        }			

			/* lanes fd det */
			if(!OPEN)
			for(int i=0; i<num_of_lanes; i++){
				mdetout[i]
				<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
				<< std::setw(digits_print) << mdetout_ensemble[i]/num_of_samples << std::endl << std::flush;
			}
			else
                        for(int i=0; i<num_of_lanes; i++){
                                mdetout[i]
                                << std::setw(digits_print) << p_enter
                                << std::setw(digits_print) << mdetout_ensemble[i]/num_of_samples << std::endl << std::flush;
                        }

			/* lanes V */
			if(!OPEN)
			for(int i=0; i<num_of_lanes; i++){
				if(num_of_times_of_lane[i]!=0){
					mVout[i]
					<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
					<< std::setw(digits_print) << mVout_ensemble[i]/num_of_times_of_lane[i] << std::endl << std::flush;
				}
			}
			else
                        for(int i=0; i<num_of_lanes; i++){
                                if(num_of_times_of_lane[i]!=0){
                                        mVout[i]
                                        << std::setw(digits_print) << p_enter
                                        << std::setw(digits_print) << mVout_ensemble[i]/num_of_times_of_lane[i] << std::endl << std::flush;
                                }
                        }			

			/* lanes d */
			if(!OPEN)
			for(int i=0; i<num_of_lanes; i++){
				mdout[i]
				<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
				<< std::setw(digits_print) << mdout_ensemble[i]/num_of_samples << std::endl << std::flush;
			}
			else
                        for(int i=0; i<num_of_lanes; i++){
                                mdout[i]
                                << std::setw(digits_print) << p_enter
                                << std::setw(digits_print) << mdout_ensemble[i]/num_of_samples << std::endl << std::flush;
                        }

			/* lane usage */
			if(!OPEN)
			for(int i=0; i<num_of_lanes; i++){
				luout[i]
				<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
				<< std::setw(digits_print) << luout_ensemble[i]/num_of_samples << std::endl << std::flush;
			}
			else
                        for(int i=0; i<num_of_lanes; i++){
                                luout[i]
                                << std::setw(digits_print) << p_enter
                                << std::setw(digits_print) << luout_ensemble[i]/num_of_samples << std::endl << std::flush;
                        }			

			/* lane usage of types */
			if(!OPEN)
			for(int i=0; i<types; i++){
				for(int j=0; j<num_of_lanes; j++){
					lutypeout[i*num_of_lanes+j]
					<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
					<< std::setw(digits_print) << lane_usage_type_ensemble[i*num_of_lanes+j]/num_of_samples << std::endl << std::flush;
				}
			}
			else
                        for(int i=0; i<types; i++){
                                for(int j=0; j<num_of_lanes; j++){
                                        lutypeout[i*num_of_lanes+j]
                                        << std::setw(digits_print) << p_enter
                                        << std::setw(digits_print) << lane_usage_type_ensemble[i*num_of_lanes+j]/num_of_samples << std::endl << std::flush;
                                }
                        }
			
			/* type fd */
			if(!OPEN)
			for(int i=0; i<types; i++){
				type_out[i]
				<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
				<< std::setw(digits_print) << type_out_ensemble[i]/num_of_samples << std::endl << std::flush;
			}
			else
                        for(int i=0; i<types; i++){
                                type_out[i]
                                << std::setw(digits_print) << p_enter
                                << std::setw(digits_print) << type_out_ensemble[i]/num_of_samples << std::endl << std::flush;
                        }


			/* type fd det */
			if(!OPEN)
			for(int i=0; i<types; i++){
				type_det_out[i]
				<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
				<< std::setw(digits_print) << type_det_out_ensemble[i]/num_of_samples << std::endl << std::flush;
			}
			else
                        for(int i=0; i<types; i++){
                                type_det_out[i]
                                << std::setw(digits_print) << p_enter
                                << std::setw(digits_print) << type_det_out_ensemble[i]/num_of_samples << std::endl << std::flush;
                        }			

			/* types V */
			if(!OPEN)
			for(int i=0; i<types; i++){
				type_Vout[i]
				<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
				<< std::setw(digits_print) << type_Vout_ensemble[i]/num_of_samples << std::endl << std::flush;
			}
			else
                        for(int i=0; i<types; i++){
                                type_Vout[i]
                                << std::setw(digits_print) << p_enter
                                << std::setw(digits_print) << type_Vout_ensemble[i]/num_of_samples << std::endl << std::flush;
                        }

			#ifdef COR_G
			/* Spatial correlation */
			for(int i=0; i<num_of_lanes; i++){
				for(int j=0; j<correlation_length; j++)
					Gout[i]
					<< std::setw(digits_print) << j
					<< std::setw(digits_print) << G_ensemble[i*correlation_length+j]/num_of_samples << std::endl << std::flush;
			}
			#endif
                        #ifdef PROFILE
                        for(int i=0; i<num_of_lanes; i++){
                                for(int j=0; j<L; j++)
                                        Rhoout[i]
                                        << std::setw(digits_print) << j
                                        << std::setw(digits_print) << Rho_ensemble[i*L+j]/num_of_samples << std::endl << std::flush;
                        }
                        #endif

			
			//Computing relaxation time tau.
			/* Relaxation time */
			if(!OPEN)
			tau_file 
			<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
			<< std::setw(digits_print) << relaxation_time(Vavg_of_time_of_all_lanes_ensemble, steps, p, Vout_ensemble/num_of_samples) << std::endl << std::flush;
			else
                        tau_file
                        << std::setw(digits_print) << p_enter
                        << std::setw(digits_print) << relaxation_time(Vavg_of_time_of_all_lanes_ensemble, steps, p, Vout_ensemble/num_of_samples) << std::endl << std::flush;

			/* number of lane changing */
			if(!OPEN)
			lcnout 
			<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
			<< std::setw(digits_print) << lane_changing_number_ensemble/num_of_samples << std::endl << std::flush;
			else
                        lcnout
                        << std::setw(digits_print) << p_enter
                        << std::setw(digits_print) << lane_changing_number_ensemble/num_of_samples << std::endl << std::flush;

			/* number of undertakings */
			if(!OPEN)
			utout 
			<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
			<< std::setw(digits_print) << num_of_undertakings_ensemble/num_of_samples << std::endl << std::flush;
			else
                        utout
                        << std::setw(digits_print) << p_enter
                        << std::setw(digits_print) << num_of_undertakings_ensemble/num_of_samples << std::endl << std::flush;			

			/* number of overtakings */
			if(!OPEN)
			otout 
			<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
			<< std::setw(digits_print) << num_of_overtakings_ensemble/num_of_samples << std::endl << std::flush;
			else
                        otout
                        << std::setw(digits_print) << p_enter
                        << std::setw(digits_print) << num_of_overtakings_ensemble/num_of_samples << std::endl << std::flush;

			
			/* order_parameter */
			if(!OPEN)
			for(int i=0; i<num_of_lanes; i++){
				order_out[i]
				<< std::setw(digits_print) << (double) N_new_ensemble/(L*num_of_lanes)
				<< std::setw(digits_print) << m_ensemble[i]/num_of_samples << std::endl << std::flush;
			}
			else
                        for(int i=0; i<num_of_lanes; i++){
                                order_out[i]
                                << std::setw(digits_print) << (double) p_enter
                                << std::setw(digits_print) << m_ensemble[i]/num_of_samples << std::endl << std::flush;
                        }
			
			/* End of file writting. */

		
		
		#ifdef COR_G
		for(int i=0; i<num_of_lanes; i++){
			(Gout+i)->close();
		}
		delete [] Gout;
		#endif

                #ifdef PROFILE
                for(int i=0; i<num_of_lanes; i++){
                        (Rhoout+i)->close();
                }
                delete [] Rhoout;
                #endif
		

		#ifdef TRACE
		
		for(int i=0; i<num_of_lanes; i++){
			(trace+i)->close();
		}
		delete [] trace;
		#endif
	}
	
	
	
	fd_discrete_out.close();
	fout.close();
	fdetout.close();
	Vout.close();
	lcnout.close();
	utout.close();
	otout.close();
#ifdef EX
	fexout.close();
	fdetexactout.close();
	Vexout.close();
	lcnexout.close();
#endif	
	
	delete [] Vmax;
	delete [] flow;
	
	
	
	
	delete [] sflow;
	#ifdef COR_G
	delete [] G;
	#endif
        #ifdef PROFILE
        delete [] Rho;
        #endif
	
	delete [] position_of_lane;
	delete [] speed_of_car;
	delete [] type_of_car;
	delete [] Vavg_of_time_of_each_lane;
	delete [] number_of_vehicle_of_time_of_each_lane;
	
	
	delete [] Vavg_of_time_of_all_lanes;
	
	delete [] Flow_over_time_of_each_lane;
	delete [] Vavg_over_time_of_each_lane;
	delete [] d_over_time_of_each_lane;
	delete [] num_of_veh_of_lane;
	delete [] num_of_fast_veh_of_lane;
	delete [] lane_type;
	delete [] lane_p;
	delete [] lane_p_enter;
	delete [] lane_p_leave;
	delete [] lane_entering_speed;
	delete [] lane_leave_range;
	
	delete [] Vavg_of_time_of_each_VehType;
	delete [] Vavg_over_time_of_each_VehType;
	
	delete [] mout_ensemble;
	delete [] mdetout_ensemble;

	delete [] mVout_ensemble;
	delete [] mdout_ensemble;
	delete [] luout_ensemble;
	delete [] type_out_ensemble;
	delete [] type_det_out_ensemble;
	delete [] type_det_exact_out_ensemble;
	delete [] type_Vout_ensemble;
	#ifdef COR_G
	delete [] G_ensemble;
	#endif
        #ifdef PROFILE
        delete [] Rho_ensemble;
        #endif
	delete [] Vavg_of_time_of_all_lanes_ensemble;
	delete [] m;
	delete [] m_ensemble;
	
	delete [] flow_lane_det;
	delete [] flow_lane_det_exact_ratio;
	delete [] lane_usage;
	delete [] lane_usage_type;
	delete [] num_of_times_of_lane;
	delete [] Vavg_token_of_lane;
	
	delete [] flow_type_det;
	delete [] flow_type_det_exact_ratio;
	
	for(int i=0; i<num_of_lanes; i++){
		(mout+i)->close();
	}
	delete [] mout;
	for(int i=0; i<num_of_lanes; i++){
		(mdetout+i)->close();
	}
	delete [] mdetout;
	for(int i=0; i<num_of_lanes; i++){
		(mVout+i)->close();
	}
	delete [] mVout;
	for(int i=0; i<num_of_lanes; i++){
		(mdout+i)->close();
	}
	delete [] mdout;
	for(int i=0; i<num_of_lanes; i++){
		(luout+i)->close();
	}
	delete [] luout;

	for(int i=0; i<types; i++){
		for(int j=0; j<num_of_lanes; j++){
			(lutypeout+i*num_of_lanes+j)->close();
		}
	}
	delete [] lutypeout;
	
	for(int i=0; i<types; i++){
		(type_out+i)->close();
	}
	delete [] type_out;
	for(int i=0; i<types; i++){
		(type_det_out+i)->close();
	}
	delete [] type_det_out;
	for(int i=0; i<types; i++){
		(type_Vout+i)->close();
	}
	delete [] type_Vout;
	for(int i=0; i<num_of_lanes; i++){
		(order_out+i)->close();
	}
	delete [] order_out;

	
	tau_file.close();
	V_f_out.close();
	V_fdet_out.close();


#ifdef MULTI
	MPI_Finalize();
#endif
	return 0;
	
}
