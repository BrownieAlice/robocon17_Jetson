static void MBdata_callback(const detect_circle::MBinput& msg);
static void usbLRF_callback(const sensor_msgs::LaserScan& msg);
static void ethLRF_callback(const sensor_msgs::LaserScan& msg);
static void Jcircle_callback(const detect_circle::Jcircle& Jcircle);
static void Jline_callback(const detect_circle::Jline& Jline);
static void write_situation(const std::string name, const bool flag);
