#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <tuple>

#define COMSIC_SPEED_LIMIT 299792458
#define C_V 299792458

using namespace std;

struct vect3 {double x; double y; double z;};
struct angvect3 {
    double roll; double pitch; double yaw;
    void add(double* field, double addition) {
        *field = fmod(abs(*field + addition), 360);
    }
};
struct vect2 {double x; double y;};

struct Mass {
    private:
        double rest_mass; 
        double rel_mass;
    public:
        Mass(double rest) {
            this->rest_mass = rest;
            this->rel_mass = rest;
        }
        Mass(double rest, double vel_magnitude) {
            this->rest_mass = rest;
            set_rel_mass(vel_magnitude);
        }
        void set_rest_mass(double new_rest_mass, double vel_magnitude) {
            this->rest_mass = new_rest_mass;
            set_rel_mass(vel_magnitude);
        }
        double get_rest_mass() {return this->rest_mass;}
        void set_rel_mass(double vel_magnitude) {
            this->rel_mass = (this->rest_mass/sqrt(1 - pow(vel_magnitude, 2) / pow(c_v, 2)));
        }
        double get_rel_mass() {return this->rel_mass;}
    };

template <typename T> struct Force {
    int index;
    string key;
    T* force;
};

class Interaction {
    public:
        /// seconds
        double time_elapsed = 0;
        double last_call = 0;
        virtual tuple<vect3, angvect3> interact(double start, double dt) = 0;
};

class ThrusterInteraction: public Interaction {
    public:
        Mover* thruster;
        Mover* parent;

        ///translation of thruster relative to parent's center
        vect3 effective_translation;
        ///rotation of thruster relative to parent's rotation
        angvect3 effective_rotation;

        bool flip;

        ThrusterInteraction(Mover* thruster, bool flip = true) {
            this->thruster = thruster;
            this->flip = flip;

            memcpy(&effective_translation, &thruster->coords, sizeof(thruster->coords));
            memcpy(&effective_rotation, &thruster->rotation, sizeof(thruster->rotation));

            if(flip == true) {
                effective_rotation.add(&effective_rotation.roll, -180);
                effective_rotation.add(&effective_rotation.pitch, -180);
                effective_rotation.add(&effective_rotation.yaw, -180);
            }

            Mover* recipient = thruster;

            while(recipient->parent != nullptr) {
                recipient = recipient->parent;

                effective_translation.x += recipient->coords.x;
                effective_translation.y += recipient->coords.y;
                effective_translation.z += recipient->coords.z;

                effective_rotation.add(&effective_rotation.roll, recipient->rotation.roll);
                effective_rotation.add(&effective_rotation.pitch, recipient->rotation.pitch);
                effective_rotation.add(&effective_rotation.yaw, recipient->rotation.yaw);
            };
            this->parent = recipient;

        }
        ~ThrusterInteraction() {
            if(thruster != parent) {
                thruster->interactions.erase(remove(thruster->interactions.begin(), thruster->interactions.end(), this), thruster->interactions.end());
                parent->interactions.erase(remove(parent->interactions.begin(), parent->interactions.end(), this), parent->interactions.end());
            } else {
                thruster->interactions.erase(remove(thruster->interactions.begin(), thruster->interactions.end(), this), thruster->interactions.end());
            }
            thruster = nullptr;
            parent = nullptr;
        }

        tuple<vect3, angvect3> interact(double start, double dt) {
            vect3* thrust_center;
            angvect3 thrust_rotation;
            if(thruster == parent) {
                memcpy(&thrust_center, &parent->coords, sizeof(vect3));
                memcpy(&thrust_rotation,  &parent->rotation, sizeof(angvect3));

                if(flip == true) {
                    thrust_rotation.add(&effective_rotation.roll, -180);
                    thrust_rotation.add(&effective_rotation.pitch, -180);
                    thrust_rotation.add(&effective_rotation.yaw, -180);
                }
            } else {
                memcpy(&thrust_center, &effective_translation, sizeof(vect3));
                memcpy(&thrust_rotation, &effective_rotation, sizeof(angvect3));
            }
            
            double thrust = thruster->current_thrust;

            vect3 F = {thrust * thrust_rotation.roll, thrust * thrust_rotation.pitch, thrust * thrust_rotation.yaw};
            angvect3 torque = {
                thrust_center->y * F.z - F.y * thrust_center->z,
                thrust_center->z * F.x - F.z * thrust_center->x,
                thrust_center->x * F.y - F.x * thrust_center->y
            };
            return {F, torque};
        }
};

//i ardly know er!
class Thruster: public Mover {
    double max_thrust = 10000;
    double current_thrust = 10000;
};

class Hellebore: public Mover {
    public:
        Hellebore(double radius, double rest_mass) : Mover(radius, rest_mass) {

        }
};

class Mover {
    public:
        Mover* parent = nullptr;
        bool is_independent() {return parent != nullptr;}
        unordered_map<string, Mover*>* components;
        unordered_map<string, Mover*>* influencers;

        vector<Interaction*> interactions;

        string name = "some mover";

        ///meters relative to (0,0,0) in this frame of reference
        vect3 coords = {0,0,0};
        ///degrees (0 roll = -y dir, 0 pitch = +x, 0 yaw = -y) relative to (0,0,0)
        angvect3 rotation = {0,0,0};

        vect3 net_force = {0,0,0};
        angvect3 net_angular_force = {0,0,0};

        double current_thrust = 0;
        double max_thrust = 0;
        //unordered_map<string, vect3*>* internal_forces;
        //unordered_map<string, vect3*>* external_forces;

        //unordered_map<string, angvect3*>* angular_forces;

        vect3 velocity = {0,0,0};
        angvect3 angular_velocity = {0,0,0};
        double radius;

        double rest_mass;
        double rel_mass;
        //Mass* mass;
        Actor* actor;
        bool has_actor = false;

        double health = 1;
        bool is_alive = true;

        double time = 0;

        Mover(double radius, double rest_mass) {
            this->radius = radius;
            set_rest_mass(rest_mass, 0);
            movers.push_back(this);
        }
        ~Mover() {
            //std::vector<int>::iterator position = find(movers.begin(), movers.end(), this);
            //if (position != movers.end()) // == myVector.end() means the element was not found
            //    movers.erase(position);
            movers.erase(remove(movers.begin(), movers.end(), this), movers.end());
            Actor* actor = this->actor;
            this->actor = NULL;
            delete actor;
        }
        Mover(const Mover& other) = delete;
        Mover& operator=(const Mover& other) = delete;
        //TODO: copy constructor and assignment operator

        tuple<vect3, angvect3> calculate_net_forces(double start, double dt) {
            vect3 net_force = {0,0,0};
            angvect3 net_torque = {0,0,0};

            for(int i = 0; i < interactions.size(); i++) {
                Interaction* interaction = interactions[i];
                auto [F, torque] = interaction->interact(start, dt);

                net_force.x += F.x; net_force.y += F.y; net_force.z += F.z;
                net_torque.roll += torque.roll; net_torque.pitch += torque.pitch; net_torque.yaw += torque.yaw;
            }

            return {net_force, net_torque};
        }

        void set_rest_mass(double new_rest_mass, double vel_magnitude) {
            this->rest_mass = new_rest_mass;
            set_rel_mass(vel_magnitude);
        }
        double get_rest_mass() {return this->rest_mass;}
        void set_rel_mass(double vel_magnitude) {
            this->rel_mass = (this->rest_mass/sqrt(1 - pow(vel_magnitude, 2) / pow(c_v, 2)));
        }
        double get_rel_mass() {return this->rel_mass;}

        void add_health(double addition) {
            set_health(this->health + addition);
        }

        void set_health(double new_health) {
            health = new_health;
            if (health <= 0) {
                set_alive(false);
            }
        }
        void set_alive(bool new_is_alive) {
            if(is_alive == new_is_alive) {
                return;
            }
            bool old_is_alive = is_alive;
            is_alive = new_is_alive;

            if (old_is_alive == true) {
                cout << "mover: " << name << " died at: " << time << "!" <<  endl;
            } else {
                cout << "mover: " << name << " was brought back to life at: " << time << "!" <<  endl;
            }
        }

        bool can_make_decisions() {
            return is_alive == true && has_actor == true;
        }
};

class Actor {

};

double cosmic_speed_limit = COMSIC_SPEED_LIMIT;
double c_v = C_V;
vector<Mover*> movers;

double get_dist(Mover* mover1, Mover* mover2) {
    vect3* center1 = &mover1->coords;
    vect3* center2 = &mover2->coords;
    return sqrt(pow(center1->x - center2->x,2) + pow(center1->y - center2->y, 2) + pow(center1->z - center2->z, 2));
}

bool is_colliding(Mover* mover1, Mover* mover2, double dt) {
    double radius1 = mover1->radius;
    double radius2 = mover2->radius;
    double distance = get_dist(mover1, mover2);
    return (distance <= radius1) || (distance <= radius2);
}

void find_collisions(double dt) {
    for(int i = 0; i < movers.size(); i++) {
        Mover* mover = movers[i];
        for(int j = i + 1; j < movers.size(); j++) {
            Mover* mover2 = movers[j];

            bool colliding = is_colliding(mover, mover2, dt);
            if (colliding == true) {
                //string mover1_coords = "{";
                //string mover2_coords = "";

                cout << mover->name << " at {" << mover->coords.x << "," << mover->coords.y << mover->coords.z << "} is colliding with " \
                    << mover2->name << " at {" << mover2->coords.x << "," << mover2->coords.y << mover2->coords.z << "}" << endl;
            }

        }
    }
}

int main() {
    vector<string> msg{"test", "test2"};
    cout << "---------- Hello World! -------------";
    
    cout << endl;
    int x = 1;
    //return 0;
}