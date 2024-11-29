#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sysmonitor_interfaces/msg/sysmon.hpp> 

// Definizione del nodo
class SystemPublisher : public rclcpp::Node
{
public:
    SystemPublisher() : Node("system_publisher") // Nome del nodo
    {
        // Creazione del publisher per il topic /test
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/test", 10);

        // Creazione del subscriber per il topic /sysmonitor
        subscriber_ = this->create_subscription<sysmonitor_interfaces::msg::Sysmon>(
            "/system_info", 10,
            std::bind(&SystemPublisher::callback_sysmonitor, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Nodo system_publisher inizializzato");
    }

private:
    // Callback chiamata quando arriva un messaggio su /sysmonitor
    void callback_sysmonitor(const sysmonitor_interfaces::msg::Sysmon::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Messaggio ricevuto su /sysmonitor");

        // Creazione di un messaggio da pubblicare su /test
        auto message = std_msgs::msg::Float64();
        message.data = 1.0; // Pubblicazione di 1.0 ogni volta che arriva un messaggio
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Pubblicato: '%f' su /test", message.data);
    }

    // Publisher per il topic /test
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;

    // Subscriber per il topic /sysmonitor
    rclcpp::Subscription<sysmonitor_interfaces::msg::Sysmon>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Inizializzo ROS 2
    rclcpp::spin(std::make_shared<SystemPublisher>()); // Eseguo il nodo
    rclcpp::shutdown(); // Arresto ROS 2
    return 0;
}
