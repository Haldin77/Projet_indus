#include <iostream>
#include <chrono>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <thread>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
// #include "omni_msgs/msg/omni_state.hpp"
// #include "omni_msgs/msg/omni_button_event.hpp"
// #include "omni_msgs/msg/omni_feedback.hpp"
#include "message.hpp"
#include "../include/ur_udp/message.hpp"

MessageUR currentMsg = {{0, 0, 0}, -1};

using namespace std;
using us = chrono::microseconds;
using get_time = chrono::steady_clock;
mutex console_mutex;

sockaddr_in serv_addr{}, cli_addr{};
socklen_t clilen;

class PhantomNode : public rclcpp::Node
{
public:
    PhantomNode() : Node("phantom_node")
    {
        // Déclaration des paramètres de configuration
        this->declare_parameter<int>("portUR", 32000);
        this->declare_parameter<std::string>("server_ip", "192.168.42.130");
        this->declare_parameter<int>("portPhantom", 32001);

        // Abonnement à un topic pour recevoir des données à envoyer à l'UR
        sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/Omni/measured_cv", 1000, std::bind(&PhantomNode::sendMsgToUR_callback, this, std::placeholders::_1));

        subpos = this->create_subscription<sensor_msgs::msg::JointState>(
            "/Omni/measured_js", 1000, std::bind(&PhantomNode::position_callback, this, std::placeholders::_1));

        // Création d'un Publisher pour envoyer des données reçues
        // pub = this->create_publisher<omni_msgs::msg::OmniFeedback>("/phantom/force_feedback", 1000);

        // sub_button = this->create_subscription<omni_msgs::msg::OmniButtonEvent>("/phantom/button",10,std::bind(&PhantomNode::button_callback, this, std::placeholders::_1));
        //  Lancer un thread pour écouter le serveur UDP
        std::thread thread_serveur_udp = std::thread(&PhantomNode::serveur_udp, this);
        /*if (thread_serveur_udp.joinable())
        {
            thread_serveur_udp.join();
        }*/
        thread_serveur_udp.detach();
    }

    ~PhantomNode() {}

private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subpos;
    // rclcpp::Publisher<omni_msgs::msg::OmniFeedback>::SharedPtr pub;
    // rclcpp::Subscription<omni_msgs::msg::OmniButtonEvent>::SharedPtr sub_button;
    std::thread thread_serveur_udp;
    std::uint32_t button = 1;
    sensor_msgs::msg::JointState omni_pos;

    // Partie serveur pour recevoir les efforts de l'UR
    void *serveur_udp()
    {
        // 1) Création de la socket
        int sockfd = socket(AF_INET, SOCK_DGRAM,0);
        if (sockfd < 0)
        {
            cerr << "Impossible d'ouvrir la socket : " << strerror(errno) << endl;
            exit(0);
        }

        // 2) Configuration de l'adresse du serveur
        memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        int portno = this->get_parameter("portPhantom").as_int();
        serv_addr.sin_port = htons(portno);

        // Liaison de la socket à l'adresse locale
        if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {
            cerr << "Impossibe de faire l'appel système bind() : " << strerror(errno) << endl;
            exit(0);
        }

        cout << "Serveur UDP en écoute sur le port " << portno << "...\n";

        char buffer[256];
        while (rclcpp::ok())
        {
            // Réinitialiser le tampon
            memset(buffer, 0, 256);
            clilen = sizeof(cli_addr);

            // Recevoir un message du client
            ssize_t received_bytes = recvfrom(sockfd, buffer, 255, MSG_DONTWAIT, (struct sockaddr *)&cli_addr, &clilen);
            if (received_bytes < 0)
            {
                if (errno != EWOULDBLOCK && errno != EAGAIN) // Si l'erreur ne correspond pas à une absence de messages
                    cerr << "Erreur lors de l'appel système recvfrom()." << strerror(errno) << endl;
                continue;
            }
            // Désérialiser le message
            msgpack::object_handle oh = msgpack::unpack(buffer, received_bytes);
            msgpack::object obj = oh.get();
            MessageUR msg;
            obj.convert(msg);

            // Duré en micro-secondes, calulée entre maintenant et ... 1970 (.time_since_epoch())
            double currentTime = chrono::duration_cast<us>(get_time::now().time_since_epoch()).count() / 1000.0;
            cout << "Temps de transmission (ms) : " << currentTime << " - " << msg.time / 1000.0 << " = " << currentTime - msg.time / 1000.0 << endl;

            // On vérifie que le message reçu n'est pas un message plus vieux que le dernier message pris en compte (sécurité sur l'ordre d'arrivée des messages)
            if ((msg.time - currentMsg.time) > 0)
            {
                // Afficher le message reçu
                //inet_ntoa() convertit l'adresse IP du client en une chaîne lisible
                //ntohs() convertit le numéro de port du client en ordre d'octets hôte
                cout << "\nNouveau message reçu de " << inet_ntoa(cli_addr.sin_addr)
                     << ":" << ntohs(cli_addr.sin_port) << "\n";
                // cout << "Vecteur de force reçu :\nx : " << msg.f.x << "\ny : " << msg.f.y << "\nz : " << msg.f.z << endl;

                currentMsg = msg;

                // omni_msgs::msg::OmniFeedback wrenchMsg;
                // wrenchMsg.force.x = msg.f.x;
                // wrenchMsg.force.y = msg.f.y;
                // wrenchMsg.force.z = msg.f.z;

                // pub->publish(wrenchMsg);
            }
        }
        close(sockfd);
    }

    void position_callback(const sensor_msgs::msg::JointState msg_pos)
    {
        omni_pos = msg_pos;
    }

    void sendMsgToUR_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg_omniState)
    {
        int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0)
        {
            cerr << "Erreur: Impossible d'ouvrir le socket." << strerror(errno) << endl;
            exit(0);
        }

        sockaddr_in serv_addr{};
        serv_addr.sin_family = AF_INET;
        int portUR = this->get_parameter("portUR").as_int();
        serv_addr.sin_port = htons(portUR);
        std::string server_ip = this->get_parameter("server_ip").as_string();
        if (inet_pton(AF_INET, server_ip.c_str(), &serv_addr.sin_addr) <= 0)
        {
            cerr << "Erreur: Adresse IP invalide." << endl;
            exit(0);
        }

        // Créer le message
        MessagePhantom::Velocity vel;
        MessagePhantom::Position pos;
        if (button == 0)
        {
            vel = {0, 0, 0};
            pos = {0, 0, 0, 0};
        }
        else
        {

            vel = {msg_omniState->twist.linear.x, msg_omniState->twist.linear.y, msg_omniState->twist.linear.z,};
            pos = {omni_pos.position[3], omni_pos.position[4], omni_pos.position[5], 0};
        }
        double time = static_cast<double>(chrono::duration_cast<us>(get_time::now().time_since_epoch()).count());
        MessagePhantom msg{vel, pos, time};

        // Sérialiser le message (= encodage)
        msgpack::sbuffer buffer;                     // Tampon pour les données sérialisées
        msgpack::pack(buffer, msg);                  // Sérialise `message` dans `buffer`
        const char *serialized_data = buffer.data(); // Accès aux données sérialisées
        std::size_t data_size = buffer.size();       // Taille des données sérialisées

        ssize_t sent_bytes = sendto(sockfd, serialized_data, data_size, 0,
                                    (struct sockaddr *)&serv_addr, sizeof(serv_addr));
        if (sent_bytes < 0)
            cerr << "Erreur lors de l'envoi du message. Code d'erreur : " << strerror(errno) << "\n";
        // geometry_msgs / msg / PoseStamped void button_callback(const omni_msgs::msg::OmniButtonEvent::SharedPtr msg)
        // {
        //     button = msg->grey_button;
        // }
        close(sockfd);
    }
};

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PhantomNode>());
    rclcpp::shutdown();

    return 0;
}
