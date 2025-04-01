#include <iostream>     // Pour les entrées/sorties standard (std::cout, std::cerr)
#include <chrono>
#include <thread>       // Pour créer et gérer des threads
#include <vector>       // Pour stocker les threads dans un vecteur
#include <cstring>      // Pour memset et fonctions de manipulation de chaînes
#include <arpa/inet.h>  // Pour les fonctions réseau (htons, inet_ntoa, etc.)
#include <unistd.h>     // Pour close() et usleep()
#include <mutex>        // Pour synchroniser les accès aux ressources partagées
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "omni_msgs/msg/omni_state.hpp"
#include "message.hpp"
#include "../include/ur_udp/message.hpp"

using namespace std;
using  us = chrono::microseconds; 
using get_time = chrono::steady_clock;

// Variable globale : vitesse reçue la plus récente
MessagePhantom currentMsg = {{0, 0, 0},{0, 0, 0, 0}, -1};

// Mutex pour synchroniser l'accès à la console pour les threads
mutex console_mutex;

pthread_t thread_serveur_udp_id;
int sockfd, portno;
sockaddr_in serv_addr{}, cli_addr{};
socklen_t clilen;

class URNode : public rclcpp::Node
{
    public:
        URNode() : Node("ur_node")
        {
            // Déclaration des paramètres de configuration
            this->declare_parameter<int>("portUR", 32000);
            this->declare_parameter<std::string>("server_ip", "192.168.42.163");
            this->declare_parameter<int>("portPhantom", 32001);
            
            // Abonnement à un topic pour recevoir des données à envoyer à l'UR
            sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
                "/wrench", 1000, std::bind(&URNode::sendMsgToPhantom_callback, this, std::placeholders::_1));

                // Création d'un Publisher pour envoyer des données reçues
            pub = this->create_publisher<omni_msgs::msg::OmniState>("/phantom_state", 1000);

            // Lancer un thread pour écouter le serveur UDP
            std::thread thread_serveur_udp = std::thread(&URNode::serveur_udp, this);
            /*if (thread_serveur_udp.joinable())
            {
                thread_serveur_udp.join();
            }*/
           thread_serveur_udp.detach();  // Laisse le thread tourner en arrière-plan
        }

        ~URNode(){}

    private:
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub;
        rclcpp::Publisher<omni_msgs::msg::OmniState>::SharedPtr pub;
        std::thread thread_serveur_udp;

        // Partie serveur pour recevoir les vitesses du Phantom
        void *serveur_udp()
        {
            // 1) Création de la socket
            sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockfd < 0) {
                cerr << "Impossible d'ouvrir la socket.\n";
                exit(0);
            }

            // 2) Configuration de l'adresse du serveur
            memset(&serv_addr, 0, sizeof(serv_addr));
            serv_addr.sin_family = AF_INET;
            serv_addr.sin_addr.s_addr = INADDR_ANY;
            int portno = this->get_parameter("portUR").as_int();
            serv_addr.sin_port = htons(portno);

            // Liaison de la socket à l'adresse locale
            if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
                cerr << "Impossibe de faire l'appel système bind().\n";
                exit(0);
            }

            std::string serverStartMsg = "Serveur UDP en écoute sur le port " + std::to_string(portno) + "...";
            RCLCPP_INFO(this->get_logger(), serverStartMsg.c_str());

            char buffer[256];
            while(rclcpp::ok())
            {
                // Réinitialiser le tampon
                memset(buffer, 0, 256);
                clilen = sizeof(cli_addr);

                // Recevoir un message du client
                ssize_t received_bytes = recvfrom(sockfd, buffer, 255, MSG_DONTWAIT, (struct sockaddr *)&cli_addr, &clilen); //MSG_DONTWAIT rend l'appel non bloquant
                if (received_bytes < 0) {
                    if(errno != EWOULDBLOCK && errno != EAGAIN) // Si l'erreur ne correspond pas à une absence de messages
                        cerr << "Erreur lors de l'appel système recvfrom()." << strerror(errno) << endl;
                    continue;
                }

                // Désérialiser le message
                msgpack::object_handle oh = msgpack::unpack(buffer, received_bytes);
                msgpack::object obj = oh.get();
                MessagePhantom msg;
                obj.convert(msg);

                //Duré en micro-secondes, calulée entre maintenant et ... 1970 (.time_since_epoch())
                double currentTime = chrono::duration_cast<us>(get_time::now().time_since_epoch()).count()/1000.0;
                cout << "Temps de transmission (ms) : " << currentTime << " - " << msg.time/1000.0 << " = " << currentTime - msg.time/1000.0 << endl;

                // On vérifie que le message reçu n'est pas un message plus vieux que le dernier message pris en compte (sécurité sur l'ordre d'arrivée des messages)
                if((msg.time - currentMsg.time) > 0)
                {
                    // Afficher le message reçu
                    // inet_ntoa() convertit l'adresse IP du client en une chaîne lisible
                    // ntohs() convertit le numéro de port du client en ordre d'octets hôte
                    cout << "\nNouveau message reçu de " << inet_ntoa(cli_addr.sin_addr)
                            << ":" << ntohs(cli_addr.sin_port) << "\n";
                    cout << "velocity reçue :\nx : " << msg.pos.x << "\ny : " << msg.pos.y << "\nvz : " << msg.pos.z << "\nwx : " << endl;

                    currentMsg = msg;

                    omni_msgs::msg::OmniState omniStateMsg;
                    omniStateMsg.velocity.x = msg.vel.vx;
                    omniStateMsg.velocity.y = msg.vel.vy;
                    omniStateMsg.velocity.z = msg.vel.vz;
                    omniStateMsg.pose.orientation.x = msg.pos.x;
                    omniStateMsg.pose.orientation.y = msg.pos.y;
                    omniStateMsg.pose.orientation.z = msg.pos.z;
                    omniStateMsg.pose.orientation.w = msg.pos.w;
                    pub->publish(omniStateMsg);
                }
            }
            close(sockfd);
        }

        void sendMsgToPhantom_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg_wrench)
        {
            RCLCPP_INFO(this->get_logger(), "Message reçu sur le topic /wrench");
            int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockfd < 0)
            {
                cerr << "Erreur: Impossible d'ouvrir le socket." << endl;
                exit(0);
            }

            sockaddr_in serv_addr{};
            serv_addr.sin_family = AF_INET;
            int portPhantom = this->get_parameter("portPhantom").as_int();
            serv_addr.sin_port = htons(portPhantom);
            std::string server_ip = this->get_parameter("server_ip").as_string();
            if (inet_pton(AF_INET, server_ip.c_str(), &serv_addr.sin_addr) <= 0)
            {
                cerr << "Erreur: Adresse IP invalide." << endl;
                exit(0);
            }

            // Créer le message
            MessageUR::Force f{msg_wrench->wrench.force.x, msg_wrench->wrench.force.y, msg_wrench->wrench.force.z};
            double time = static_cast<double>(chrono::duration_cast<us>(get_time::now().time_since_epoch()).count());
            MessageUR msg{f, time};

            // Sérialiser le message (= encodage)
            msgpack::sbuffer buffer;                      // Tampon pour les données sérialisées
            msgpack::pack(buffer, msg);               // Sérialise `message` dans `buffer`
            const char* serialized_data = buffer.data();  // Accès aux données sérialisées
            std::size_t data_size = buffer.size();        // Taille des données sérialisées

            ssize_t sent_bytes = sendto(sockfd, serialized_data, data_size, 0,
                                            (struct sockaddr *)&serv_addr, sizeof(serv_addr));
            if (sent_bytes < 0)
                cerr << "Erreur lors de l'envoi du message. Code d'erreur : " << strerror(errno) << "\n";
            
            close(sockfd);
        }
};

int main(int argc, char *argv[]) {
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<URNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
