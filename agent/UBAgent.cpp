#include "UBAgent.h"

#include "config.h"
#include "QsLog.h"

#include "UBNetwork.h"
#include "UBVision.h"

#include "UASManager.h"
#include "LinkManager.h"
#include "ArduPilotMegaMAV.h"
#include "LinkManagerFactory.h"

#include "mercatorprojection.h"

UBAgent::UBAgent(QObject *parent) : QObject(parent),
    m_uav(NULL),
    m_mission_stage(STAGE_IDLE)
{
    m_net = new UBNetwork(this);
    m_sensor = new UBVision(this);
    connect(m_net, SIGNAL(dataReady()), this, SLOT(dataReadyEvent()));

    m_timer = new QTimer(this);
    m_timer->setInterval(MISSION_TRACK_RATE);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(missionTracker()));
}

void UBAgent::startAgent() {
    int port = MAV_PORT;

    int idx = QCoreApplication::arguments().indexOf("--port");
    if (idx > 0)
        port = QCoreApplication::arguments().at(idx + 1).toInt();

    int link = 0;
//    link = LinkManagerFactory::addSerialConnection(SERIAL_PORT, BAUD_RATE);
    link = LinkManagerFactory::addTcpConnection(QHostAddress::LocalHost, "", port, false);

    LinkManager::instance()->connectLink(link);
    connect(UASManager::instance(), SIGNAL(UASCreated(UASInterface*)), this, SLOT(UASCreatedEvent(UASInterface*)));
}

void UBAgent::UASCreatedEvent(UASInterface* uav) {
    if (m_uav)
        return;

    m_uav = qobject_cast<ArduPilotMegaMAV*>(uav);

    if (!m_uav)
        return;

    m_uav->setHeartbeatEnabled(true);

    connect(m_uav, SIGNAL(armed()), this, SLOT(armedEvent()));
    connect(m_uav, SIGNAL(disarmed()), this, SLOT(disarmedEvent()));
    connect(m_uav, SIGNAL(navModeChanged(int,int,QString)), this, SLOT(navModeChangedEvent(int,int)));

    int port = MAV_PORT;

    int idx = QCoreApplication::arguments().indexOf("--port");
    if (idx > 0)
        port = QCoreApplication::arguments().at(idx + 1).toInt();

    m_net->startNetwork(m_uav->getUASID(), (PHY_PORT - MAV_PORT) + port);
    m_sensor->startSensor((SNR_PORT - MAV_PORT) + port);

//    QTimer::singleShot(START_DELAY, m_uav, SLOT(armSystem()));
    m_mission_data.maxID = m_uav->getUASID();
    m_timer->start();
}

void UBAgent::armedEvent() {
    m_mission_data.wps.clear();
    m_mission_data.idx = m_uav->getUASID();
    m_mission_data.wps = m_uav->getWaypointManager()->getWaypointEditableList();

    if ((m_mission_data.wps.count() < (m_mission_data.idx + 2)) || (m_mission_data.wps[m_mission_data.idx]->getAction() != MAV_CMD_NAV_WAYPOINT) || (m_mission_data.wps[m_mission_data.idx + 1]->getAction() != MAV_CMD_NAV_WAYPOINT))
        return;

//    if (m_uav->getGroundSpeed() > 1)
//        return;

//    if (!inPointZone(m_uav->getLatitude(), m_uav->getLongitude(), 0))
//        return;

    if (m_uav->getSatelliteCount() < GPS_ACCURACY)
        return;

    if (m_uav->getCustomMode() != ApmCopter::GUIDED)
        m_uav->setMode(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, ApmCopter::GUIDED);

//    m_uav->executeCommand(MAV_CMD_MISSION_START, 1, 0, 0, 0, 0, 0, 0, 0, 0);
    m_uav->executeCommand(MAV_CMD_NAV_TAKEOFF, 1, 0, 0, 0, 0, 0, 0, TAKEOFF_ALT, 0);

    m_mission_stage = STAGE_BEGIN;
}

void UBAgent::disarmedEvent() {
    m_mission_stage = STAGE_IDLE;
}

void UBAgent::navModeChangedEvent(int uasID, int mode) {
    if (uasID != m_uav->getUASID())
        return;

    if (mode == ApmCopter::GUIDED)
        return;

    if (m_mission_stage != STAGE_IDLE) {
        m_mission_stage = STAGE_IDLE;
        QLOG_WARN() << "Mission Interrupted!";
    }
}

void UBAgent::dataReadyEvent() {
    QByteArray data = m_net->getData();

    if (!data.count())
        return;

    if (data.count() == 1) {
        int id = data.data()[0];
        if (m_mission_data.maxID < id)
            m_mission_data.maxID = id;

        return;
    }

//    if (m_uav->getCustomMode() != ApmCopter::GUIDED)
//        m_uav->setMode(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, ApmCopter::GUIDED);

//    if (!m_uav->isArmed())
//        m_uav->armSystem();
}

double UBAgent::distance(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2) {
   double x1, y1, z1;
   double x2, y2, z2;

   projections::MercatorProjection proj;

   proj.FromGeodeticToCartesian(lat1, lon1, alt1, x1, y1, z1);
   proj.FromGeodeticToCartesian(lat2, lon2, alt2, x2, y2, z2);

   return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

bool UBAgent::inPointZone(double lat, double lon, double alt) {
    double dist = distance(m_uav->getLatitude(), m_uav->getLongitude(), m_uav->getAltitudeRelative(), lat, lon, alt);

    if (dist < POINT_ZONE)
        return true;

    return false;
}

void UBAgent::missionTracker() {
//    if (m_start_time < START_DELAY) {
//        m_start_time++;
//        return;
//    }

    switch (m_mission_stage) {
    case STAGE_IDLE:
        stageIdle();
        break;
    case STAGE_BEGIN:
        stageBegin();
        break;
    case STAGE_MISSION:
        stageMission();
        break;
    case STAGE_END:
        stageEnd();
        break;
    default:
        break;
    }
}

void UBAgent::stageIdle() {
    m_uav->getWaypointManager()->readWaypoints(true);
    m_net->sendData(BROADCAST_ADDRESS, QByteArray(1, m_uav->getUASID()));
}

void UBAgent::stageBegin() {
    if (inPointZone(m_uav->getLatitude(), m_uav->getLongitude(), TAKEOFF_ALT)) {
        m_mission_data.reset();
        m_mission_stage = STAGE_MISSION;

        QLOG_INFO() << "Mission Begin";
    }
}

void UBAgent::stageEnd() {
    m_uav->setMode(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, ApmCopter::RTL);
//    m_uav->land();

    m_mission_stage = STAGE_IDLE;
    QLOG_INFO() << "Mission End";
}

void UBAgent::stageMission() {
    static double lat, lon;
    static double step, step1, step2, coef;
    static Vector3d v, v0, v1, v2, n, n0, n1, n2;

    projections::MercatorProjection proj;

    double dist = distance(m_uav->getLatitude(), m_uav->getLongitude(), 0, m_mission_data.wps[m_mission_data.idx + 1]->getLatitude(), m_mission_data.wps[m_mission_data.idx + 1]->getLongitude(), 0);

    if (dist < VISUAL_RANGE) {
        m_mission_data.idx += m_mission_data.maxID;

        if ((m_mission_data.wps.count() < (m_mission_data.idx + 2)) || (m_mission_data.wps[m_mission_data.idx]->getAction() != MAV_CMD_NAV_WAYPOINT) || (m_mission_data.wps[m_mission_data.idx + 1]->getAction() != MAV_CMD_NAV_WAYPOINT)) {
            m_mission_stage = STAGE_END;
            return;
        } else {
            m_mission_data.reset();
            QLOG_INFO() << "Search Zone Changed";
        }
    }

    if (m_mission_data.stage == 0) {
        m_mission_data.stage++;

//        core::Point pix0 = proj.FromLatLngToPixel(m_mission_data.wps[0]->getLatitude(), m_mission_data.wps[0]->getLongitude(), GND_RES);
//        core::Point pix1 = proj.FromLatLngToPixel(m_mission_data.wps[m_mission_data.idx]->getLatitude(), m_mission_data.wps[m_mission_data.idx]->getLongitude(), GND_RES);
//        core::Point pix2 = proj.FromLatLngToPixel(m_mission_data.wps[m_mission_data.idx + 1]->getLatitude(), m_mission_data.wps[m_mission_data.idx + 1]->getLongitude(), GND_RES);

//        v0 = Vector3d(pix0.X(), pix0.Y(), 0);
//        v1 = Vector3d(pix1.X(), pix1.Y(), 0);
//        v2 = Vector3d(pix2.X(), pix2.Y(), 0);

        double x0, y0, z0;
        double x1, y1, z1;
        double x2, y2, z2;

        proj.FromGeodeticToCartesian(m_mission_data.wps[0]->getLatitude(), m_mission_data.wps[0]->getLongitude(), 0, x0, y0, z0);
        proj.FromGeodeticToCartesian(m_mission_data.wps[m_mission_data.idx]->getLatitude(), m_mission_data.wps[m_mission_data.idx]->getLongitude(), 0, x1, y1, z1);
        proj.FromGeodeticToCartesian(m_mission_data.wps[m_mission_data.idx + 1]->getLatitude(), m_mission_data.wps[m_mission_data.idx + 1]->getLongitude(), 0, x2, y2, z2);

        v0 = Vector3d(x0, y0, z0);
        v1 = Vector3d(x1, y1, z1);
        v2 = Vector3d(x2, y2, z2);

        n0 = v1 - v0;
        n1 = v2 - v1;
        n2 = v2 - v0;

        n0 = (1 / n0.length()) * n0;
        n1 = (1 / n1.length()) * n1;
        n2 = (1 / n2.length()) * n2;

        double dist = sqrt(pow(VISUAL_RANGE, 2) - pow(m_uav->getAltitudeRelative(), 2)) - 1;
//        double res = proj.GetGroundResolution(GND_RES, m_uav->getLatitude());
        double res = 1;
        step1 = (dist / res) / sin(acos(Vector3d::dotProduct(n0, n1)));
        step2 = (dist / res) / sin(acos(Vector3d::dotProduct(n0, n2)));

        v = v0;
        n = n2;
        step = step2;
        coef = 1;

        Vector3d pix = coef * step * n + v;

//        internals::PointLatLng pll = proj.FromPixelToLatLng(pix.x(), pix.y(), GND_RES);
//        lat = pll.Lat();
//        lon = pll.Lng();

        proj.FromCartesianTGeodetic(pix.x(), pix.y(), pix.z(), lat, lon);

        Waypoint wp;
        wp.setFrame(MAV_FRAME_GLOBAL_RELATIVE_ALT);
        wp.setLatitude(lat);
        wp.setLongitude(lon);
        wp.setAltitude(m_uav->getAltitudeRelative());

        m_uav->getWaypointManager()->goToWaypoint(&wp);

        return;
    }

    if (inPointZone(lat, lon, m_uav->getAltitudeRelative())) {
          m_mission_data.tick++;

          if (m_mission_data.tick % 2) {
              if (v == v1) {
                  v = v0;
                  n = n2;
                  step = step2;
              } else {
                  v = v1;
                  n = n1;
                  step = step1;
              }
          } else {
              coef += 2;
          }

          Vector3d pix = coef * step * n + v;

//          internals::PointLatLng pll = proj.FromPixelToLatLng(pix.x(), pix.y(), GND_RES);
//          lat = pll.Lat();
//          lon = pll.Lng();

          proj.FromCartesianTGeodetic(pix.x(), pix.y(), pix.z(), lat, lon);

          Waypoint wp;
          wp.setFrame(MAV_FRAME_GLOBAL_RELATIVE_ALT);
          wp.setLatitude(lat);
          wp.setLongitude(lon);
          wp.setAltitude(m_uav->getAltitudeRelative());

          m_uav->getWaypointManager()->goToWaypoint(&wp);

          return;
      }
}
