#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include <random>
#include <chrono>
#include <cmath>

struct Particle {
    float x, y, z;
    float vx, vy, vz;
};

class ParticleLifeNode : public rclcpp::Node {
public:
    ParticleLifeNode()
    : Node("particle_life_node")
    {
        // --- パラメータ宣言 ---
        declare_parameter("num_particles", 200);
        declare_parameter("update_rate", 50); // ms
        declare_parameter("world_radius", 10.0);
        declare_parameter("neighbor_radius", 1.5);
        declare_parameter("max_speed", 0.2);
        declare_parameter("separation_weight", 0.05);
        declare_parameter("alignment_weight", 0.05);
        declare_parameter("cohesion_weight", 0.01);

        get_parameter("num_particles", num_particles_);
        get_parameter("update_rate", update_rate_);
        get_parameter("world_radius", world_radius_);
        get_parameter("neighbor_radius", neighbor_radius_);
        get_parameter("max_speed", max_speed_);
        get_parameter("separation_weight", separation_weight_);
        get_parameter("alignment_weight", alignment_weight_);
        get_parameter("cohesion_weight", cohesion_weight_);

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("particles", 10);

        // --- 粒子初期化 ---
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(-world_radius_, world_radius_);
        std::uniform_real_distribution<float> vel_dis(-max_speed_/2.0, max_speed_/2.0);

        for (size_t i = 0; i < num_particles_; i++) {
            Particle p;
            p.x = dis(gen);
            p.y = dis(gen);
            p.z = dis(gen);
            p.vx = vel_dis(gen);
            p.vy = vel_dis(gen);
            p.vz = vel_dis(gen);
            particles_.push_back(p);
        }

        // --- タイマー ---
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(update_rate_),
            std::bind(&ParticleLifeNode::update, this)
        );
    }

private:
    void update() {
        std::vector<Particle> new_particles = particles_;

        for (size_t i = 0; i < particles_.size(); i++) {
            auto &p = particles_[i];
            float sx = 0, sy = 0, sz = 0; // separation
            float ax = 0, ay = 0, az = 0; // alignment
            float cx = 0, cy = 0, cz = 0; // cohesion
            int neighbor_count = 0;

            for (size_t j = 0; j < particles_.size(); j++) {
                if (i == j) continue;
                auto &q = particles_[j];
                float dx = q.x - p.x;
                float dy = q.y - p.y;
                float dz = q.z - p.z;
                float dist2 = dx*dx + dy*dy + dz*dz;
                if (dist2 < neighbor_radius_*neighbor_radius_) {
                    float dist = std::sqrt(dist2);

                    // Separation (反発)
                    if (dist > 0)
                    {
                        sx -= (dx / dist);
                        sy -= (dy / dist);
                        sz -= (dz / dist);
                    }

                    // Alignment (整列)
                    ax += q.vx;
                    ay += q.vy;
                    az += q.vz;

                    // Cohesion (凝集)
                    cx += q.x;
                    cy += q.y;
                    cz += q.z;

                    neighbor_count++;
                }
            }

            if (neighbor_count > 0) {
                ax /= neighbor_count;
                ay /= neighbor_count;
                az /= neighbor_count;

                cx = (cx / neighbor_count - p.x);
                cy = (cy / neighbor_count - p.y);
                cz = (cz / neighbor_count - p.z);
            }

            // 速度更新
            float vx = p.vx + separation_weight_*sx + alignment_weight_*ax + cohesion_weight_*cx;
            float vy = p.vy + separation_weight_*sy + alignment_weight_*ay + cohesion_weight_*cy;
            float vz = p.vz + separation_weight_*sz + alignment_weight_*az + cohesion_weight_*cz;

            // 最大速度制限
            float speed = std::sqrt(vx*vx + vy*vy + vz*vz);
            if (speed > max_speed_) {
                vx = vx / speed * max_speed_;
                vy = vy / speed * max_speed_;
                vz = vz / speed * max_speed_;
            }

            // 範囲制限（球状）
            if (std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z) > world_radius_) {
                vx -= 0.1 * p.x;
                vy -= 0.1 * p.y;
                vz -= 0.1 * p.z;
            }

            new_particles[i].vx = vx;
            new_particles[i].vy = vy;
            new_particles[i].vz = vz;

            new_particles[i].x += vx;
            new_particles[i].y += vy;
            new_particles[i].z += vz;
        }

        particles_ = new_particles;

        // --- PointCloud2 作成 ---
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "map";
        cloud_msg.height = 1;
        cloud_msg.width = particles_.size();
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(particles_.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (auto &p : particles_) {
            *iter_x = p.x; ++iter_x;
            *iter_y = p.y; ++iter_y;
            *iter_z = p.z; ++iter_z;
        }

        publisher_->publish(cloud_msg);
    }

    // --- メンバ変数 ---
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Particle> particles_;

    // パラメータ
    size_t num_particles_;
    int update_rate_;
    float world_radius_;
    float neighbor_radius_;
    float max_speed_;
    float separation_weight_;
    float alignment_weight_;
    float cohesion_weight_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleLifeNode>());
    rclcpp::shutdown();
    return 0;
}
