/*  Ground Terrain Collision Avoidance 
    things to do:
    1. understand DAIDALUS
    2. read other related papers and form a summary of sorts
    3. code the high-level functions first and sort out the lower-level details later

*/

#include<stdio.h>
#include<stdlib.h>
#include<vector>
#include<ros/ros.h>
// #include<std_msgs>
#include<math.h>
using namespace std;
#define R_EARTH 6378137
void straight_flight_predict(float, float, float);
void turning_flight_predict(float, float);
void search_volume_predict()

//phi_pred0 and lambda_pred0 are the aircraft position
double phi_pred0 = 0, lambda_pred0 = 0;
float psi_c = 0;
// taking one sample time to be 0.01 seconds
const float del_t_pred = 0.01, t_t = psi_c;

void straight_flight_predict(const float t_pred, float horizontal_vel, float altitude){
    //t_pred = Look ahead time
    int N = t_pred/del_t_pred, phi_ac, lambda_ac;
    float d_step = del_t_pred*horizontal_vel/(R_EARTH + altitude);
    vector<float>* phi_pred, lambda_pred;
    //initialising the aircraft positions
    phi_pred.push_back(0);
    lambda_pred.push_back(0);
    for(int i=1; i<N; i++){
        //position update for straight flight
        phi_pred.push_back(
            phi_ac + asin(sin(phi_ac)*cos(i*d_step) + cos(phi_ac)*sin(i*d_step)*cos(t_t))
        );
        lambda_pred.push_back(
            lambda_ac + atan(sin(t_t)*sin(i*d_step)*cos(phi_ac)/(cos(i*d_step)-sin(phi_ac)*sin(phi_pred.back())))
        );
    }   
}


void turning_flight_predict(float horizontal_vel, float altitude){
    float psi_ac = phi_pred0, lambda_ac = lambda_pred0, t_t = psi_c;
    vector<float> psi_pred, lambda_pred;
    float alpha, delta_psi;
    float dx[2] = {0, 0};
    int i = 0;
    //delta_psi->aircraft's yaw rate
    //alpha->centre angle

    psi_pred.push_back(0);
    lambda_pred.push_back(0);
    
    while(0){
        alpha = psi_c - M_PI/2;
        psi_c = psi_c + delta_psi*del_t_pred;

        dx[0] = (horizontal_vel*(-sin(alpha))*del_t_pred) - 0.5*horizontal_vel*delta_psi*cos(alpha)*del_t_pred*del_t_pred;
        dx[1] = (horizontal_vel*(-cos(alpha))*del_t_pred) - 0.5*horizontal_vel*delta_psi*sin(alpha)*del_t_pred*del_t_pred;

        float dist_dx = pow(dx[0]*dx[0] + dx[1]*dx[1], 0.5);

        //new predicted position
        float a = dist_dx/(R_EARTH+altitude);
        float b = atan(dx[1]/dx[0]);

        if(i>0 && i<51){
            psi_pred.push_back(psi_pred[i-1]+asin(psi_pred[i-1]*cos(a) + cos(psi_pred[i-1])*sin(a)*cos(b)));
            lambda_pred.push_back(lambda_pred[i-1]+atan((sin(b)*sin(a)*cos(psi_pred[i-1]))/(cos(a)-sin(psi_pred[i-1]*sin(psi_pred[i])))));
        }
        else{
            psi_pred.erase(psi_pred.begin());
        }

        i++;
    }
}

void search_volume_predict(){

}


int main(int argc, char** argv){

    ros::init(argc, argv, "tcas");
    ros::NodeHandle node_handle;
    ros::Publisher publisher = node_handle.advertise<std_msgs::String>("tcas_topic", 10);
    while(ros::ok()){

    }

    return 0;
}


// SRTM DATA EXTRACTION:
// def get_sample(filename, n, e):
//     i = 1201 - int(round(n / 3, 0))
//     j = int(round(e / 3, 0))
//     with open(filename, "rb") as f:
//         f.seek(((i - 1) * 1201 + (j - 1)) * 2)  # go to the right spot,
//         buf = f.read(2)  # read two bytes and convert them:
//         val = struct.unpack('>h', buf)  # ">h" is a signed two byte integer
//         if not val == -32768:  # the not-a-valid-sample value
//             return val
//         else:
//             return None

// if __name__ == "__main__":
//     # elevation at 18°27'17.0"N 73°55'22.1"E
//     n = 27 * 60 + 0
//     e = 55 * 60 + 22.1
//     tile = "/home/sagar/Desktop/UAV-Avionics/srtm data/N18E073.hgt"
//     print(get_sample(tile, n, e))