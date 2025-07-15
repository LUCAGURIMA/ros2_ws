#include <Eigen/Geometry> // Biblioteca para manipulação de quaternions e ângulos de Euler

// ROS e dependências
#include "pose_estimation.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#ifdef cv_bridge_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Biblioteca AprilTag
#include "tag_functions.hpp"
#include <apriltag.h>

// Macro para facilitar atribuição de parâmetros
#define IF(N, V) \
    if(assign_check(parameter, N, V)) continue;

// Funções auxiliares para atribuição de parâmetros
template<typename T>
void assign(const rclcpp::Parameter& parameter, T& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
void assign(const rclcpp::Parameter& parameter, std::atomic<T>& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
bool assign_check(const rclcpp::Parameter& parameter, const std::string& name, T& var)
{
    if(parameter.get_name() == name) {
        assign(parameter, var);
        return true;
    }
    return false;
}

// Função para criar descritor de parâmetro
rcl_interfaces::msg::ParameterDescriptor
descr(const std::string& description, const bool& read_only = false)
{
    rcl_interfaces::msg::ParameterDescriptor descr;
    descr.description = description;
    descr.read_only = read_only;
    return descr;
}

// Classe principal do nó AprilTag
class AprilTagNode : public rclcpp::Node {
public:
    // Construtor: inicializa parâmetros, subscrições e publishers
    AprilTagNode(const rclcpp::NodeOptions& options);

    // Destrutor: libera recursos do detector
    ~AprilTagNode() override;

private:
    // Callback para atualização dinâmica de parâmetros
    const OnSetParametersCallbackHandle::SharedPtr cb_parameter;

    // Detector e família de tags
    apriltag_family_t* tf;
    apriltag_detector_t* const td;

    // Parâmetros e variáveis de configuração
    std::mutex mutex;
    double tag_edge_size;
    std::atomic<int> max_hamming;
    std::atomic<bool> profile;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;

    // Função para destruir família de tags
    std::function<void(apriltag_family_t*)> tf_destructor;

    // Subscriber da imagem da câmera (com info de calibração)
    const image_transport::CameraSubscriber sub_cam;

    // Publisher do array de detecções (TOPICO: "detections")
    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;

    // Publisher de transformações TF
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // Função para estimar pose do tag
    pose_estimation_f estimate_pose = nullptr;

    // Callback principal: processa cada frame da câmera
    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    // Callback para atualização de parâmetros
    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);
};

// Macro para registrar o nó como componente ROS 2
RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)

// Implementação do construtor
AprilTagNode::AprilTagNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    // Callback para parâmetros dinâmicos
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagNode::onParameter, this, std::placeholders::_1))),
    // Criação do detector
    td(apriltag_detector_create()),
    // Subscrição da imagem da câmera
    sub_cam(image_transport::create_camera_subscription(
        this,
        "image_rect",
        std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2),
        declare_parameter("image_transport", "raw", descr({}, true)),
        rmw_qos_profile_sensor_data)),
    // Publisher do array de detecções
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
    // Publisher de TF
    tf_broadcaster(this)
{
    // Parâmetros de configuração do nó
    const std::string tag_family = declare_parameter("family", "36h11", descr("tag family", true));
    tag_edge_size = declare_parameter("size", 1.0, descr("default tag size", true));

    // Parâmetros de tags específicos
    const auto ids = declare_parameter("tag.ids", std::vector<int64_t>{}, descr("tag ids", true));
    const auto frames = declare_parameter("tag.frames", std::vector<std::string>{}, descr("tag frame names per id", true));
    const auto sizes = declare_parameter("tag.sizes", std::vector<double>{}, descr("tag sizes per id", true));

    // Parâmetro do método de estimação de pose
    const std::string& pose_estimation_method =
        declare_parameter("pose_estimation_method", "pnp",
                          descr("pose estimation method: \"pnp\" (more accurate) or \"homography\" (faster), "
                                "set to \"\" (empty) to disable pose estimation",
                                true));

    // Seleção do método de estimação de pose
    if(!pose_estimation_method.empty()) {
        if(pose_estimation_methods.count(pose_estimation_method)) {
            estimate_pose = pose_estimation_methods.at(pose_estimation_method);
        }
        else {
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown pose estimation method '" << pose_estimation_method << "'.");
        }
    }

    // Parâmetros do detector
    declare_parameter("detector.threads", td->nthreads, descr("number of threads"));
    declare_parameter("detector.decimate", td->quad_decimate, descr("decimate resolution for quad detection"));
    declare_parameter("detector.blur", td->quad_sigma, descr("sigma of Gaussian blur for quad detection"));
    declare_parameter("detector.refine", td->refine_edges, descr("snap to strong gradients"));
    declare_parameter("detector.sharpening", td->decode_sharpening, descr("sharpening of decoded images"));
    declare_parameter("detector.debug", td->debug, descr("write additional debugging images to working directory"));

    declare_parameter("max_hamming", 0, descr("reject detections with more corrected bits than allowed"));
    declare_parameter("profile", false, descr("print profiling information to stdout"));

    // Configuração dos frames e tamanhos específicos dos tags
    if(!frames.empty()) {
        if(ids.size() != frames.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and frames (" + std::to_string(frames.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_frames[ids[i]] = frames[i]; }
    }

    if(!sizes.empty()) {
        if(ids.size() != sizes.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and sizes (" + std::to_string(sizes.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_sizes[ids[i]] = sizes[i]; }
    }

    // Criação da família de tags
    if(tag_fun.count(tag_family)) {
        tf = tag_fun.at(tag_family).first();
        tf_destructor = tag_fun.at(tag_family).second;
        apriltag_detector_add_family(td, tf);
    }
    else {
        throw std::runtime_error("Unsupported tag family: " + tag_family);
    }
}

// Destrutor: libera recursos do detector e da família de tags
AprilTagNode::~AprilTagNode()
{
    apriltag_detector_destroy(td);
    tf_destructor(tf);
}

// Callback principal: processa cada imagem recebida da câmera
void AprilTagNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci)
{
    // Extrai os parâmetros intrínsecos da câmera
    const std::array<double, 4> intrinsics = {msg_ci->p[0], msg_ci->p[5], msg_ci->p[2], msg_ci->p[6]};

    // Verifica se a câmera está calibrada
    const bool calibrated = msg_ci->width && msg_ci->height &&
                            intrinsics[0] && intrinsics[1] && intrinsics[2] && intrinsics[3];

    // Aviso se pose estimation estiver habilitado mas a câmera não calibrada
    if(estimate_pose != nullptr && !calibrated) {
        RCLCPP_WARN_STREAM(get_logger(), "The camera is not calibrated! Set 'pose_estimation_method' to \"\" (empty) to disable pose estimation and this warning.");
    }

    // Converte a imagem para escala de cinza 8 bits
    const cv::Mat img_uint8 = cv_bridge::toCvShare(msg_img, "mono8")->image;

    // Cria estrutura de imagem para o detector
    image_u8_t im{img_uint8.cols, img_uint8.rows, img_uint8.cols, img_uint8.data};

    // Detecta tags na imagem
    mutex.lock();
    zarray_t* detections = apriltag_detector_detect(td, &im);
    mutex.unlock();

    // Exibe informações de profiling se habilitado
    if(profile)
        timeprofile_display(td->tp);

    // Cria mensagem para publicar array de detecções
    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = msg_img->header;

    // Vetor para armazenar transformações TF
    std::vector<geometry_msgs::msg::TransformStamped> tfs;

    // Loop sobre todas as detecções encontradas
    for(int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        // Debug: imprime informações da detecção
        RCLCPP_DEBUG(get_logger(),
                     "detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                     i, det->family->nbits, det->family->h, det->id,
                     det->hamming, det->decision_margin);

        // Ignora tags não rastreados (se configurado)
        if(!tag_frames.empty() && !tag_frames.count(det->id)) { continue; }

        // Ignora detecções com hamming acima do permitido
        if(det->hamming > max_hamming) { continue; }

        // Cria mensagem de detecção individual
        apriltag_msgs::msg::AprilTagDetection msg_detection;
        msg_detection.family = std::string(det->family->name); // Família do tag
        msg_detection.id = det->id; // ID do tag
        msg_detection.hamming = det->hamming; // Correção de bits
        msg_detection.decision_margin = det->decision_margin; // Confiança
        msg_detection.centre.x = det->c[0]; // Centro X
        msg_detection.centre.y = det->c[1]; // Centro Y
        std::memcpy(msg_detection.corners.data(), det->p, sizeof(double) * 8); // Cantos
        std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double) * 9); // Homografia

        // Estima pose 3D se habilitado e calibrado
        if(estimate_pose != nullptr && calibrated) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header = msg_img->header;
            // Define nome do frame filho
            tf.child_frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : std::string(det->family->name) + ":" + std::to_string(det->id);
            const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size;
            tf.transform = estimate_pose(det, intrinsics, size);
            tfs.push_back(tf);

            // Converte quaternion para ângulos de Euler (roll, pitch, yaw)
            Eigen::Quaterniond q(
                tf.transform.rotation.w,
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z
            );
            Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2); // Ordem: X, Y, Z

            // Salva ângulos em graus
            msg_detection.roll = euler[2] * 180.0 / M_PI;
            msg_detection.pitch = euler[1] * 180.0 / M_PI;
            msg_detection.yaw = euler[0] * 180.0 / M_PI;
            // Salva profundidade Z
            msg_detection.z = tf.transform.translation.z;
        } else {
            // Se não for possível estimar pose, zera os campos
            msg_detection.roll = 0.0;
            msg_detection.pitch = 0.0;
            msg_detection.yaw = 0.0;
            msg_detection.z = 0.0;
        }

        // Adiciona detecção ao array para publicação
        msg_detections.detections.push_back(msg_detection);
    }

    // PUBLICAÇÃO: envia array de detecções para o tópico "detections"
    pub_detections->publish(msg_detections);

    // PUBLICAÇÃO: envia transformações TF se pose estimation estiver habilitado
    if(estimate_pose != nullptr)
        tf_broadcaster.sendTransform(tfs);

    // Libera memória das detecções
    apriltag_detections_destroy(detections);
}

// Callback para atualização dinâmica de parâmetros do nó
rcl_interfaces::msg::SetParametersResult
AprilTagNode::onParameter(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;

    mutex.lock();

    for(const rclcpp::Parameter& parameter : parameters) {
        RCLCPP_DEBUG_STREAM(get_logger(), "setting: " << parameter);

        IF("detector.threads", td->nthreads)
        IF("detector.decimate", td->quad_decimate)
        IF("detector.blur", td->quad_sigma)
        IF("detector.refine", td->refine_edges)
        IF("detector.sharpening", td->decode_sharpening)
        IF("detector.debug", td->debug)
        IF("max_hamming", max_hamming)
        IF("profile", profile)
    }

    mutex.unlock();

    result.successful = true;

    return result;aaaa
}