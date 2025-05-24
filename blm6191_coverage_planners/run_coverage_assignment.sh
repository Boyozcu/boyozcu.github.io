#!/bin/bash
# set -x 

echo "--- Script Başlangıcı: Ortam Ayarlanıyor ---"
ROS_DISTRO_FOR_SOURCE_SCRIPT=${ROS_DISTRO:-noetic}
SETUP_BASH_SOURCED_CORRECTLY=false

if [ -f "/opt/ros/${ROS_DISTRO_FOR_SOURCE_SCRIPT}/setup.bash" ]; then
    . "/opt/ros/${ROS_DISTRO_FOR_SOURCE_SCRIPT}/setup.bash"
    SETUP_BASH_SOURCED_CORRECTLY=true
fi

SCRIPT_DIR_FOR_SOURCE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_DIR_FOR_SOURCE=$(cd "$SCRIPT_DIR_FOR_SOURCE/../../" && pwd)

if [ -f "${WORKSPACE_DIR_FOR_SOURCE}/devel/setup.bash" ]; then
    . "${WORKSPACE_DIR_FOR_SOURCE}/devel/setup.bash"
    SETUP_BASH_SOURCED_CORRECTLY=true
else
    echo "UYARI: Çalışma alanı ${WORKSPACE_DIR_FOR_SOURCE}/devel/setup.bash bulunamadı."
fi

if ! $SETUP_BASH_SOURCED_CORRECTLY ; then
    echo "KRİTİK UYARI: Hiçbir ROS ortamı source edilemedi."
fi
echo "--- Ortam Ayarlama Tamamlandı ---"
sleep 1

# === Konfigürasyon Başlangıcı ===
ROS_DISTRO_DETECTED=$(echo $ROS_DISTRO)
export ROS_DISTRO=${ROS_DISTRO_DETECTED:-noetic}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
BASE_WORKSPACE_DIR=$(cd "$SCRIPT_DIR/../../" && pwd)

ROS_PACKAGE_NAME="blm6191_coverage_planners"
LOG_DIR_RELATIVE_PATH="src/${ROS_PACKAGE_NAME}/logs"
LOG_DIR="${BASE_WORKSPACE_DIR}/${LOG_DIR_RELATIVE_PATH}"

export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-waffle}

GAZEBO_WORLD_LAUNCH_FILE="turtlebot3_gazebo turtlebot3_willowgarage.launch"
SLAM_LAUNCH_FILE="turtlebot3_slam turtlebot3_slam.launch"
SLAM_METHOD="gmapping"

RVIZ_CONFIG_FILE_NAME="coverage_assignment.rviz"
RVIZ_CONFIG_FILE_PATH="${SCRIPT_DIR}/${RVIZ_CONFIG_FILE_NAME}"

CUSTOM_MOVE_BASE_LAUNCH_FILE_NAME="my_coverage_move_base.launch"
CUSTOM_MOVE_BASE_LAUNCH_FILE_PATH="${SCRIPT_DIR}/${CUSTOM_MOVE_BASE_LAUNCH_FILE_NAME}"

CFG_robot_radius="0.20" # Duvarlara daha az yaklaşması için biraz artırıldı
CFG_sweep_spacing_factor="0.7" # Örtüşmeyi artır, boşluk kalma riskini azalt
CFG_path_point_distance="0.08"
CFG_lookahead_distance="0.5" # Takip için
CFG_linear_vel="0.10"
CFG_max_angular_vel="0.5"
CFG_goal_dist_tolerance="0.15"

CFG_use_param_poly="true"
CFG_launch_polygon_publisher="false"
CFG_launch_polygon_visualizer="true"

# Willow Garage haritasına göre ortadaki büyük masayı ve etrafını kapsayacak şekilde
# Daha geniş bir alan tanımlandı. Haritanızdaki koordinatlara göre ayarlayın!
# Örneğin, X: -1.5 ile 1.5 arası, Y: -2.0 ile 1.0 arası gibi bir alan.
CFG_p1x="-1.5"; CFG_p1y="1.0"  # Sol Üst
CFG_p2x="1.5";  CFG_p2y="1.0"  # Sağ Üst
CFG_p3x="1.5";  CFG_p3y="-2.0" # Sağ Alt
CFG_p4x="-1.5"; CFG_p4y="-2.0" # Sol Alt

PERFORM_CATKIN_MAKE_AT_START=false
# === Konfigürasyon Sonu ===

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
mkdir -p "$LOG_DIR"
LOG_FILE="${LOG_DIR}/run_log_${TIMESTAMP}.txt"

declare -A PIDS
PIDS=( [roscore]=0 [gazebo]=0 [slam]=0 [rviz]=0 [move_base]=0 [poly_viz]=0 )

log_message() {
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] $1" | tee -a "$LOG_FILE"
}

cleanup() {
    log_message "#####################################################"
    log_message "Temizleme sinyali alındı, süreçler sonlandırılıyor..."
    for process_name in gazebo slam rviz move_base poly_viz; do
        pid=${PIDS[$process_name]}
        if [[ $pid -ne 0 && -e /proc/$pid ]]; then
            log_message "$process_name (PID: $pid) sonlandırılıyor (SIGINT)..."
            kill -SIGINT $pid
        fi
    done
    sleep 2 

    for process_name in gazebo slam rviz move_base poly_viz; do
        pid=${PIDS[$process_name]}
        if [[ $pid -ne 0 && -e /proc/$pid ]]; then
            log_message "$process_name (PID: $pid) hala çalışıyor, SIGKILL gönderiliyor..."
            kill -SIGKILL $pid; sleep 0.5
        fi
        log_message "$process_name sonlandırıldı (veya zaten kapalıydı)."
    done
    
    if [[ ${PIDS[roscore]} -ne 0 && -e /proc/${PIDS[roscore]} ]]; then
        log_message "Bu script tarafından başlatılan roscore (PID: ${PIDS[roscore]}) sonlandırılıyor..."
        kill -SIGINT ${PIDS[roscore]}; sleep 1
        if [[ -e /proc/${PIDS[roscore]} ]]; then kill -SIGKILL ${PIDS[roscore]}; fi
        log_message "Roscore sonlandırıldı."
    fi

    log_message "Temizleme tamamlandı. Log dosyası: $LOG_FILE"
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "=====================================================" > "$LOG_FILE"
log_message "KAPSAMA PLANLAYICI ÖDEV SCRIPT'İ BAŞLATILIYOR"
# ... (diğer log mesajları)
log_message "Otomatik Poligon: P1(${CFG_p1x},${CFG_p1y}), P2(${CFG_p2x},${CFG_p2y}), P3(${CFG_p3x},${CFG_p3y}), P4(${CFG_p4x},${CFG_p4y})"
log_message "====================================================="
sleep 1

if ! pgrep -x "rosmaster" > /dev/null && ! pgrep -x "roscore" > /dev/null ; then
    (roscore) >> "$LOG_FILE" 2>&1 &
    PIDS[roscore]=$!
    log_message "Roscore PID: ${PIDS[roscore]}. Başlaması için 5 sn bekleniyor..."
    sleep 5
    if ! pgrep -x "rosmaster" > /dev/null && ! pgrep -x "roscore" > /dev/null ; then
        log_message "HATA: Roscore başlatılamadı!"; cleanup; exit 1
    fi
    log_message "Roscore başarıyla başlatıldı."
else
    PIDS[roscore]=0 
    log_message "Mevcut roscore çalışıyor. PID: $(pgrep -x rosmaster || pgrep -x roscore)"
fi

if [ "$PERFORM_CATKIN_MAKE_AT_START" = true ]; then
    log_message "Çalışma alanı derleniyor..."
    cd "$BASE_WORKSPACE_DIR" || { log_message "HATA: Çalışma alanına geçilemedi."; cleanup; exit 1; }
    if catkin_make -DCATKIN_WHITELIST_PACKAGES="$ROS_PACKAGE_NAME" >> "$LOG_FILE" 2>&1; then
        log_message "Derleme başarılı."
    else
        log_message "HATA: Derleme başarısız oldu."; cleanup; exit 1
    fi
    if [ -f "${BASE_WORKSPACE_DIR}/devel/setup.bash" ]; then
        . "${BASE_WORKSPACE_DIR}/devel/setup.bash"
    else
        log_message "HATA: Derleme sonrası setup.bash bulunamadı!"; cleanup; exit 1;
    fi
fi

log_message "Gazebo (${GAZEBO_WORLD_LAUNCH_FILE}) başlatılıyor..."
(roslaunch $GAZEBO_WORLD_LAUNCH_FILE) >> "$LOG_FILE" 2>&1 &
PIDS[gazebo]=$!
log_message "Gazebo PID: ${PIDS[gazebo]}. Yüklenmesi için 15 sn bekleniyor..."
sleep 15

log_message "SLAM (${SLAM_LAUNCH_FILE} - Yöntem: ${SLAM_METHOD}) başlatılıyor..."
(roslaunch ${SLAM_LAUNCH_FILE} slam_methods:="${SLAM_METHOD}" open_rviz:=false) >> "$LOG_FILE" 2>&1 &
PIDS[slam]=$!
log_message "SLAM PID: ${PIDS[slam]}. Başlaması için 8 sn bekleniyor..."
sleep 8

if [ -f "$RVIZ_CONFIG_FILE_PATH" ]; then
    (rosrun rviz rviz -d "$RVIZ_CONFIG_FILE_PATH") >> "$LOG_FILE" 2>&1 &
else
    (rosrun rviz rviz) >> "$LOG_FILE" 2>&1 &
fi
PIDS[rviz]=$!
log_message "RVIZ PID: ${PIDS[rviz]}. Başlaması için 5 sn bekleniyor..."
sleep 5

if [ "$CFG_launch_polygon_visualizer" = "true" ]; then
    log_message "Polygon Visualizer başlatılıyor..."
    (rosrun "$ROS_PACKAGE_NAME" polygon_visualizer_node.py \
        _p1x:="$CFG_p1x" _p1y:="$CFG_p1y" \
        _p2x:="$CFG_p2x" _p2y:="$CFG_p2y" \
        _p3x:="$CFG_p3x" _p3y:="$CFG_p3y" \
        _p4x:="$CFG_p4x" _p4y:="$CFG_p4y" \
        _frame_id:="map" \
    ) >> "$LOG_FILE" 2>&1 &
    PIDS[poly_viz]=$!
    log_message "Polygon Visualizer PID: ${PIDS[poly_viz]}. Başlaması için 2 sn bekleniyor..."
    sleep 2
fi

log_message "Navigasyon (move_base - ${CUSTOM_MOVE_BASE_LAUNCH_FILE_NAME}) başlatılıyor..."
(roslaunch "$ROS_PACKAGE_NAME" "$CUSTOM_MOVE_BASE_LAUNCH_FILE_NAME" \
    model:="$TURTLEBOT3_MODEL" \
    robot_radius_val:="$CFG_robot_radius" \
    sweep_spacing_factor_val:="$CFG_sweep_spacing_factor" \
    path_point_distance_val:="$CFG_path_point_distance" \
    lookahead_distance_val:="$CFG_lookahead_distance" \
    linear_vel_val:="$CFG_linear_vel" \
    max_angular_vel_val:="$CFG_max_angular_vel" \
    goal_dist_tolerance_val:="$CFG_goal_dist_tolerance" \
    use_param_poly_val:="$CFG_use_param_poly" \
    launch_polygon_publisher_val:="$CFG_launch_polygon_publisher" \
    launch_polygon_visualizer_val:="$CFG_launch_polygon_visualizer" \
    p1x_val:="$CFG_p1x" p1y_val:="$CFG_p1y" \
    p2x_val:="$CFG_p2x" p2y_val:="$CFG_p2y" \
    p3x_val:="$CFG_p3x" p3y_val:="$CFG_p3y" \
    p4x_val:="$CFG_p4x" p4y_val:="$CFG_p4y" \
) >> "$LOG_FILE" 2>&1 &
PIDS[move_base]=$!
log_message "Move_base launch PID: ${PIDS[move_base]}. Başlaması için 10 sn bekleniyor..."
sleep 10

log_message "#####################################################"
log_message "SİSTEM BAŞLATILDI. LÜTFEN AŞAĞIDAKİ ADIMLARI İZLEYİN:"
log_message "1. Robotu manuel olarak (teleop ile) Gazebo ortamında gezdirerek harita oluşturun."
log_message "   Yeni terminalde: source ${BASE_WORKSPACE_DIR}/devel/setup.bash && roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch"
log_message "2. RViz'de haritanın oluştuğunu ve TF ağacının (map->odom->base_footprint) kurulduğunu gözlemleyin."
log_message "3. RViz'de kapsama alanı poligonunun (mavi çizgiler) doğru çizildiğini kontrol edin."
log_message "4. Harita yeterli olduktan sonra, RViz'deki '2D Nav Goal' aracıyla bir hedef belirleyin."
log_message "5. Robotun kapsama yolunu oluşturup takip ettiğini gözlemleyin."
log_message "#####################################################"
log_message "Ana süreçler çalışıyor. Sonlandırmak için bu terminalde Ctrl+C yapın."

while true; do
    critical_processes_running=false
    if [[ ${PIDS[gazebo]} -ne 0 && -e /proc/${PIDS[gazebo]} ]]; then critical_processes_running=true; fi
    if [[ ${PIDS[slam]} -ne 0 && -e /proc/${PIDS[slam]} ]]; then critical_processes_running=true; fi

    if ! $critical_processes_running && ! pgrep -x "gzserver" > /dev/null ; then
        log_message "Kritik süreçler (Gazebo, SLAM) veya gzserver sonlanmış."
        if ! pgrep -x "rosmaster" > /dev/null && ! pgrep -x "roscore" > /dev/null ; then
           log_message "Roscore/Rosmaster da sonlanmış. Script temizlenerek çıkacak."
           break
        fi
    fi
    
    if ! pgrep -x "rosmaster" > /dev/null && ! pgrep -x "roscore" > /dev/null ; then
        log_message "Roscore/Rosmaster sonlanmış. Script temizlenerek çıkacak."
        break
    fi
    sleep 30
done

cleanup
