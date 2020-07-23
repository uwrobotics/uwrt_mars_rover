#include "rqt_gps/gpsconv_plugin.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QFormLayout>
#include <stdio.h>


#include <rqt_gps/DDGPS.h>
#include <rqt_gps/DMSGPS.h>
#include <vector>

namespace rqt_gps
{

GPSConvPlugin::GPSConvPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function

  // give QObjects reasonable names
  setObjectName("GPSConvPlugin");
}

void GPSConvPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);

  //GPS Coordinate Publishers
  DDGPS_pub = n.advertise<rqt_gps::DDGPS>("DDGPS_coords", 1000);
  DMSGPS_pub = n.advertise<rqt_gps::DMSGPS>("DMSGPS_coords", 1000);

  //Parent Layout Reference
  QVBoxLayout *parentLayout = widget_->findChild<QVBoxLayout*>("parentLayout");

  //Button References
  QPushButton *ddbutton = widget_->findChild<QPushButton*>("ddButton");
  QPushButton *dmsbutton = widget_->findChild<QPushButton*>("dmsButton");
  QPushButton *ddPublishButton = widget_->findChild<QPushButton*>("ddPublishButton");
  QPushButton *dmsPublishButton = widget_->findChild<QPushButton*>("dmsPublishButton");

  //Add events for button activations
  connect(ddbutton, SIGNAL (released()), this, SLOT (handleDDButton()));
  connect(dmsbutton, SIGNAL (released()), this, SLOT (handleDMSButton()));
  connect(ddPublishButton, SIGNAL (released()), this, SLOT (handleDDPublishButton()));
  connect(dmsPublishButton, SIGNAL (released()), this, SLOT (handleDMSPublishButton()));

  widget_->setLayout(parentLayout);

  // add widget to the user interface
  context.addWidget(widget_);
}

void GPSConvPlugin::handleDDButton(){
  /*
  * Handles the decimal degree conversion event (converting from DMS to DD)
  * */

  //Get references to line edits with entered coordinates
  QLineEdit *latDeg = widget_->findChild<QLineEdit*>("latDeg");
  QLineEdit *latMin = widget_->findChild<QLineEdit*>("latMin");
  QLineEdit *latSec = widget_->findChild<QLineEdit*>("latSec");
  QLineEdit *lonDeg = widget_->findChild<QLineEdit*>("lonDeg");
  QLineEdit *lonMin = widget_->findChild<QLineEdit*>("lonMin");
  QLineEdit *lonSec = widget_->findChild<QLineEdit*>("lonSec");

  //Get references to line edits to output results
  QLineEdit *latDec = widget_->findChild<QLineEdit*>("latDec");
  QLineEdit *lonDec = widget_->findChild<QLineEdit*>("lonDec");

  //Converts entered coordinates to doubles
  double latD = latDeg->text().toDouble();
  double latM = latMin->text().toDouble();
  double latS = latSec->text().toDouble();
  double lonD = lonDeg->text().toDouble();
  double lonM = lonMin->text().toDouble();
  double lonS = lonSec->text().toDouble();
  
  //Converts doubles to two vector for longitude and latitude (ease of passing data)
  std::vector<double> inputLat = {latD, latM, latS};
  std::vector<double> inputLon = {lonD, lonM, lonS};

  //Call conversion function and return a vector of the result
  std::vector<double> convCoord = dmsToDD(inputLat, inputLon);

  //Display result in textboxes
  //NOTE: Must change the sigdig to be output
  latDec->setText(QString("%1").arg(convCoord.at(0)));
  lonDec->setText(QString("%1").arg(convCoord.at(1)));
}

void GPSConvPlugin::handleDDPublishButton(){
  /*
  * Handles the decimal degree publishing event
  * */

  //Get references to decimal degree entered coordinates
  QLineEdit *latDec = widget_->findChild<QLineEdit*>("latDec");
  QLineEdit *lonDec = widget_->findChild<QLineEdit*>("lonDec");

  //Convert the values to doubles
  double latD = latDec->text().toDouble();
  double latM = lonDec->text().toDouble();

  //Publish the Decimal Degree custom message
  rqt_gps::DDGPS msg;
  msg.latitude = latD;
  msg.longitude = latM;
  DDGPS_pub.publish(msg);
}

std::vector<double> GPSConvPlugin::dmsToDD(std::vector<double> inputLat, std::vector<double> inputLon){
  /*
  * Converts deg, min, sec coodinate vectors to decimal coordinate vector
  */

  //Simple math formula, nothing crazy here
  double lat = inputLat.at(0) + (inputLat.at(1)/60) + (inputLat.at(2)/3600);
  double lon = inputLon.at(0) + (inputLon.at(1)/60) + (inputLon.at(2)/3600);
  std::vector<double> out = {lat, lon};
  return out;
}

void GPSConvPlugin::handleDMSButton(){
   /*
  * Handles the degree, minute, second conversion event (converting from DD to DMS)
  * */

  //Get references to line edits with entered coordinates
  QLineEdit *latDec = widget_->findChild<QLineEdit*>("latDec");
  QLineEdit *lonDec = widget_->findChild<QLineEdit*>("lonDec");

  //Get references to line edits to output results
  QLineEdit *latDeg = widget_->findChild<QLineEdit*>("latDeg");
  QLineEdit *latMin = widget_->findChild<QLineEdit*>("latMin");
  QLineEdit *latSec = widget_->findChild<QLineEdit*>("latSec");
  QLineEdit *lonDeg = widget_->findChild<QLineEdit*>("lonDeg");
  QLineEdit *lonMin = widget_->findChild<QLineEdit*>("lonMin");
  QLineEdit *lonSec = widget_->findChild<QLineEdit*>("lonSec");

  //Converts entered coordinates to doubles
  double latDD = latDec->text().toDouble();
  double lonDD = lonDec->text().toDouble();

  //Call conversion function and return a vector of the result
  std::vector<double> convCoord = ddToDMS(latDD, lonDD);


  //Display result in textboxes
  //NOTE: Must change the sigdig to be output
  latDeg->setText(QString("%1").arg(convCoord.at(0)));
  latMin->setText(QString("%1").arg(convCoord.at(1)));
  latSec->setText(QString("%1").arg(convCoord.at(2)));
  lonDeg->setText(QString("%1").arg(convCoord.at(3)));
  lonMin->setText(QString("%1").arg(convCoord.at(4)));
  lonSec->setText(QString("%1").arg(convCoord.at(5)));
}

void GPSConvPlugin::handleDMSPublishButton(){
  /*
  * Handles the degree, minute, second publishing event
  * */

  //Get references to degree, minute, second entered coordinates
  QLineEdit *latDeg = widget_->findChild<QLineEdit*>("latDeg");
  QLineEdit *latMin = widget_->findChild<QLineEdit*>("latMin");
  QLineEdit *latSec = widget_->findChild<QLineEdit*>("latSec");
  QLineEdit *lonDeg = widget_->findChild<QLineEdit*>("lonDeg");
  QLineEdit *lonMin = widget_->findChild<QLineEdit*>("lonMin");
  QLineEdit *lonSec = widget_->findChild<QLineEdit*>("lonSec");

  //Convert the values to doubles
  double latD = latDeg->text().toDouble();
  double latM = latMin->text().toDouble();
  double latS = latSec->text().toDouble();
  double lonD = lonDeg->text().toDouble();
  double lonM = lonMin->text().toDouble();
  double lonS = lonSec->text().toDouble();

  //Publish the Degree Minute Second custom message
  rqt_gps::DMSGPS msg;
  msg.latitudeDeg = latD;
  msg.latitudeMin = latM;
  msg.latitudeSec = latS;
  msg.longitudeDeg = lonD;
  msg.longitudeMin = lonM;
  msg.longitudeSec = lonS;
  DMSGPS_pub.publish(msg);
}

std::vector<double> GPSConvPlugin::ddToDMS(double inputLat, double inputLon){
  /*
  *  Converts decimal coodinate vectors to deg, min, sec coordinate vector
  */

  //Just a reversal of the DMS to DD formula
  //Using mod to ensure the only non-integer values that show up are seconds
  double r1 = std::fmod(inputLat,1.0);
  double latDeg = inputLat - r1;
  double r2 = std::fmod((r1 *60),1.0);
  double latMin = (r1*60) - r2;
  double latSec = r2 * 60;

  double r3 = std::fmod(inputLon,1.0);
  double lonDeg = inputLon - r3;
  double r4 = std::fmod((r3 *60),1.0);
  double lonMin = (r3*60)- r4;
  double lonSec = r4 * 60;

  std::vector<double> out = {latDeg,latMin,latSec,lonDeg,lonMin,lonSec};
  return out;
}


/*Below are some functions that were included in some of the sample and tutorial
* files I scraped. Possibly useful in the future depending on what we need to do so
* I left them here.
*/
void GPSConvPlugin::shutdownPlugin()
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

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

}  // namespace rqt_gps

//Export Plugin 
PLUGINLIB_EXPORT_CLASS(rqt_gps::GPSConvPlugin, rqt_gui_cpp::Plugin)

