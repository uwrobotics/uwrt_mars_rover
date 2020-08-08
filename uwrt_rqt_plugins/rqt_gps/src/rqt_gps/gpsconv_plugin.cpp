#include "rqt_gps/gpsconv_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QStringList>
#include <QVBoxLayout>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <rqt_gps/DDGPS.h>
#include <rqt_gps/DMSGPS.h>
#include <rqt_gps/DDGPSarray.h>
#include <rqt_gps/DMSGPSarray.h>

#include <vector>

namespace rqt_gps {

GPSConvPlugin::GPSConvPlugin() {
  // Constructor is called first before initPlugin function
  // give QObjects reasonable names
  setObjectName("GPSConvPlugin");
}

void GPSConvPlugin::initPlugin(qt_gui_cpp::PluginContext &context) {
  const int message_queue = 1000;
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);

  // GPS Coordinate Publishers
  DDGPS_pub = n.advertise<rqt_gps::DDGPSarray>("DDGPS_coords", message_queue);
  DMSGPS_pub = n.advertise<rqt_gps::DMSGPSarray>("DMSGPS_coords", message_queue);

  // Parent Layout Reference
  auto *parent_layout = widget_->findChild<QVBoxLayout *>("parent_layout");

  // Button References
  auto *dd_button = widget_->findChild<QPushButton *>("ddButton");
  auto *dms_button = widget_->findChild<QPushButton *>("dmsButton");
  auto *dd_add_button = widget_->findChild<QPushButton *>("ddAddButton");
  auto *dms_add_button = widget_->findChild<QPushButton *>("dmsAddButton");
  auto *dd_rem_button = widget_->findChild<QPushButton *>("ddRemButton");
  auto *dms_rem_button = widget_->findChild<QPushButton *>("dmsRemButton");
  auto *dd_publish_button = widget_->findChild<QPushButton *>("ddPublishButton");
  auto *dms_publish_button = widget_->findChild<QPushButton *>("dmsPublishButton");

  // Add events for button activations
  connect(dd_button, SIGNAL(released()), this, SLOT(handleDDButton()));
  connect(dms_button, SIGNAL(released()), this, SLOT(handleDMSButton()));
  connect(dd_add_button, SIGNAL(released()), this, SLOT(handleDDAddButton()));
  connect(dms_add_button, SIGNAL(released()), this, SLOT(handleDMSAddButton()));
  connect(dd_rem_button, SIGNAL(released()), this, SLOT(handleDDRemButton()));
  connect(dms_rem_button, SIGNAL(released()), this, SLOT(handleDMSRemButton()));
  connect(dd_publish_button, SIGNAL(released()), this, SLOT(handleDDPublishButton()));
  connect(dms_publish_button, SIGNAL(released()), this, SLOT(handleDMSPublishButton()));

  widget_->setLayout(parent_layout);

  // add widget to the user interface
  context.addWidget(widget_);
}

void GPSConvPlugin::handleDDButton() {
  /*
   * Handles the decimal degree conversion event (converting from DMS to DD)
   * */

  // Get references to line edits with entered coordinates
  auto *lat_deg = widget_->findChild<QLineEdit *>("latDeg");
  auto *lat_min = widget_->findChild<QLineEdit *>("latMin");
  auto *lat_sec = widget_->findChild<QLineEdit *>("latSec");
  auto *lon_deg = widget_->findChild<QLineEdit *>("lonDeg");
  auto *lon_min = widget_->findChild<QLineEdit *>("lonMin");
  auto *lon_sec = widget_->findChild<QLineEdit *>("lonSec");

  // Get references to line edits to output results
  auto *lat_dec = widget_->findChild<QLineEdit *>("latDec");
  auto *lon_dec = widget_->findChild<QLineEdit *>("lonDec");

  // Converts entered coordinates to doubles
  double lat_d = lat_deg->text().toDouble();
  double lat_m = lat_min->text().toDouble();
  double lat_s = lat_sec->text().toDouble();
  double lon_d = lon_deg->text().toDouble();
  double lon_m = lon_min->text().toDouble();
  double lon_s = lon_sec->text().toDouble();

  // Converts doubles to two vector for longitude and latitude (ease of passing data)
  std::vector<double> input_lat = {lat_d, lat_m, lat_s};
  std::vector<double> input_lon = {lon_d, lon_m, lon_s};

  // Call conversion function and return a vector of the result
  std::vector<double> conv_coord = dmsToDD(input_lat, input_lon);

  // Display result in textboxes
  // NOTE: Must change the sigdig to be output
  const int lat_pos = 0;
  const int lon_pos = 1;
  lat_dec->setText(QString("%1").arg(conv_coord.at(lat_pos)));
  lon_dec->setText(QString("%1").arg(conv_coord.at(lon_pos)));
}

void GPSConvPlugin::handleDDAddButton() {
  /*
   * Handles the decimal degree conversion event (converting from DMS to DD)
   * */
  // Get references to line edits to output results
  auto *lat_dec = widget_->findChild<QLineEdit *>("latDec");
  auto *lon_dec = widget_->findChild<QLineEdit *>("lonDec");

  // Converts entered coordinates to doubless
  double lat_d = lat_dec->text().toDouble();
  double lon_d = lon_dec->text().toDouble();

  // Call conversion function and return a vector of the result
  std::vector<double> conv_coord = {lat_d,lon_d};

  DDList.push_back(conv_coord);
}

void GPSConvPlugin::handleDDRemButton() {
  DDList.pop_back();
}

void GPSConvPlugin::handleDDPublishButton() {
  /*
   * Handles the decimal degree publishing event
   * */

  // Get references to decimal degree entered coordinates
  auto *lat_dec = widget_->findChild<QLineEdit *>("latDec");
  auto *lon_dec = widget_->findChild<QLineEdit *>("lonDec");

  // Convert the values to doubles
  double lat_d = lat_dec->text().toDouble();
  double lon_d = lon_dec->text().toDouble();

  // Publish the Decimal Degree custom message
  rqt_gps::DDGPSarray msgarr;
  rqt_gps::DDGPS coord;
  std::vector<rqt_gps::DDGPS> temp = {};
  // for(int i=0; i<DDList.size(); i++){
  //   msg
  // }
  for(int i=0; i<DDList.size(); i++){
    coord.latitude = DDList.at(i)[0];
    coord.longitude = DDList.at(i)[1];
    temp.push_back(coord);
  }
  msgarr.coord_array = temp;
  DDGPS_pub.publish(msgarr);
}

std::vector<double> GPSConvPlugin::dmsToDD(std::vector<double> inputLat, std::vector<double> inputLon) {
  /*
   * Converts deg, min, sec coodinate vectors to decimal coordinate vector
   */
  const int minute_factor = 60;
  const int second_factor = 3600;

  // Simple math formula, nothing crazy here
  double lat = inputLat.at(0) + (inputLat.at(1) / minute_factor) + (inputLat.at(2) / second_factor);
  double lon = inputLon.at(0) + (inputLon.at(1) / minute_factor) + (inputLon.at(2) / second_factor);
  std::vector<double> out = {lat, lon};
  return out;
}

void GPSConvPlugin::handleDMSButton() {
  /*
   * Handles the degree, minute, second conversion event (converting from DD to DMS)
   * */

  // Get references to line edits with entered coordinates
  auto *lat_dec = widget_->findChild<QLineEdit *>("latDec");
  auto *lon_dec = widget_->findChild<QLineEdit *>("lonDec");

  // Get references to line edits to output results
  auto *lat_deg = widget_->findChild<QLineEdit *>("latDeg");
  auto *lat_min = widget_->findChild<QLineEdit *>("latMin");
  auto *lat_sec = widget_->findChild<QLineEdit *>("latSec");
  auto *lon_deg = widget_->findChild<QLineEdit *>("lonDeg");
  auto *lon_min = widget_->findChild<QLineEdit *>("lonMin");
  auto *lon_sec = widget_->findChild<QLineEdit *>("lonSec");

  // Converts entered coordinates to doubles
  double lat_dd = lat_dec->text().toDouble();
  double lon_dd = lon_dec->text().toDouble();

  // Call conversion function and return a vector of the result
  std::vector<double> conv_coord = ddToDMS(lat_dd, lon_dd);

  // Display result in textboxes
  const int latd_pos = 0;
  const int latm_pos = 1;
  const int lats_pos = 2;

  const int lond_pos = 3;
  const int lonm_pos = 4;
  const int lons_pos = 5;
  // NOTE: Must change the sigdig to be output
  lat_deg->setText(QString("%1").arg(conv_coord.at(latd_pos)));
  lat_min->setText(QString("%1").arg(conv_coord.at(latm_pos)));
  lat_sec->setText(QString("%1").arg(conv_coord.at(lats_pos)));
  lon_deg->setText(QString("%1").arg(conv_coord.at(lond_pos)));
  lon_min->setText(QString("%1").arg(conv_coord.at(lonm_pos)));
  lon_sec->setText(QString("%1").arg(conv_coord.at(lons_pos)));
}

void GPSConvPlugin::handleDMSAddButton() {
  /*
   * Handles the decimal degree conversion event (converting from DMS to DD)
   * */
  // Get references to degree, minute, second entered coordinates
  auto *lat_deg = widget_->findChild<QLineEdit *>("latDeg");
  auto *lat_min = widget_->findChild<QLineEdit *>("latMin");
  auto *lat_sec = widget_->findChild<QLineEdit *>("latSec");
  auto *lon_deg = widget_->findChild<QLineEdit *>("lonDeg");
  auto *lon_min = widget_->findChild<QLineEdit *>("lonMin");
  auto *lon_sec = widget_->findChild<QLineEdit *>("lonSec");

  // Convert the values to doubles
  double lat_d = lat_deg->text().toDouble();
  double lat_m = lat_min->text().toDouble();
  double lat_s = lat_sec->text().toDouble();
  double lon_d = lon_deg->text().toDouble();
  double lon_m = lon_min->text().toDouble();
  double lon_s = lon_sec->text().toDouble();


  // Call conversion function and return a vector of the result
  std::vector<double> conv_coord = {lat_d,lat_m,lat_s,lon_d,lon_m,lon_s};

  DMSList.push_back(conv_coord);
}

void GPSConvPlugin::handleDMSRemButton() {
  DMSList.pop_back();
}

void GPSConvPlugin::handleDMSPublishButton() {
  /*
   * Handles the degree, minute, second publishing event
   * */

  // Get references to degree, minute, second entered coordinates
  auto *lat_deg = widget_->findChild<QLineEdit *>("latDeg");
  auto *lat_min = widget_->findChild<QLineEdit *>("latMin");
  auto *lat_sec = widget_->findChild<QLineEdit *>("latSec");
  auto *lon_deg = widget_->findChild<QLineEdit *>("lonDeg");
  auto *lon_min = widget_->findChild<QLineEdit *>("lonMin");
  auto *lon_sec = widget_->findChild<QLineEdit *>("lonSec");

  // Convert the values to doubles
  double lat_d = lat_deg->text().toDouble();
  double lat_m = lat_min->text().toDouble();
  double lat_s = lat_sec->text().toDouble();
  double lon_d = lon_deg->text().toDouble();
  double lon_m = lon_min->text().toDouble();
  double lon_s = lon_sec->text().toDouble();

  // Publish the Degree Minute Second custom message
    // Publish the Decimal Degree custom message
  rqt_gps::DMSGPSarray msgarr;
  rqt_gps::DMSGPS coord;
  std::vector<rqt_gps::DMSGPS> temp = {};
  // for(int i=0; i<DDList.size(); i++){
  //   msg
  // }
  for(int i=0; i<DMSList.size(); i++){
    coord.latitudeDeg = DMSList.at(i)[0];
    coord.latitudeMin = DMSList.at(i)[1];
    coord.latitudeSec = DMSList.at(i)[2];
    coord.longitudeDeg = DMSList.at(i)[3];
    coord.longitudeMin = DMSList.at(i)[4];
    coord.longitudeSec = DMSList.at(i)[5];
    temp.push_back(coord);
  }
  msgarr.coord_array = temp;
  DMSGPS_pub.publish(msgarr);
}

std::vector<double> GPSConvPlugin::ddToDMS(double inputLat, double inputLon) {
  /*
   *  Converts decimal coodinate vectors to deg, min, sec coordinate vector
   */

  // Just a reversal of the DMS to DD formula
  // Using mod to ensure the only non-integer values that show up are seconds
  const double second_factor = 60;

  double r1 = std::fmod(inputLat, 1.0);
  double lat_deg = inputLat - r1;
  double r2 = std::fmod((r1 * second_factor), 1.0);
  double lat_min = (r1 * second_factor) - r2;
  double lat_sec = r2 * second_factor;

  double r3 = std::fmod(inputLon, 1.0);
  double lon_deg = inputLon - r3;
  double r4 = std::fmod((r3 * second_factor), 1.0);
  double lon_min = (r3 * second_factor) - r4;
  double lon_sec = r4 * second_factor;

  std::vector<double> out = {lat_deg, lat_min, lat_sec, lon_deg, lon_min, lon_sec};
  return out;
}

/*Below are some functions that were included in some of the sample and tutorial
 * files I scraped. Possibly useful in the future depending on what we need to do so
 * I left them here.
 */
/*void GPSConvPlugin::shutdownPlugin()
{
  // unregister all publishers here
}

void GPSConvPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void GPSConvPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

}  // namespace rqt_gps

// Export Plugin
PLUGINLIB_EXPORT_CLASS(rqt_gps::GPSConvPlugin, rqt_gui_cpp::Plugin)
