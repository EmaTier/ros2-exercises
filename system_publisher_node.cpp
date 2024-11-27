#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

//ogni nodo Ros2 deve essere una sottoclasse di Node
class SystemPublisher : public rclcpp::Node
{
public:
//inizializzo il nodo con il nome system_publisher
    SystemPublisher() : Node("system_publisher") //costruttore
    {
        // Creo un publisher per il topic /test
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/test", 10);

        // Creo un timer per pubblicare ogni secondo
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SystemPublisher::publish_data, this)
        );
    }

private:
    void publish_data()
    {
        // Creo un nuovo messaggio di tipo Float64
        auto message = std_msgs::msg::Float64();

        // Impost il valore che voglio pubblicare
        message.data = 10.0; 

        // Pubblico il messaggio
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
        publisher_->publish(message);
    }

    //puntatore condiviso al publisher (pubblica messaggi di tipo std_msgs::msg::Float64)
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    //puntatore condiviso al timer (esegue la funzione publish_data ogni secondo)
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); //inizializzo ros2
    rclcpp::spin(std::make_shared<SystemPublisher>());//avvio il nodo creando un oggetto della classe systemPublisher
    rclcpp::shutdown(); //arresto ros2
    return 0;
}
