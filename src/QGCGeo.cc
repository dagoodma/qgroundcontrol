/*=====================================================================

 QGroundControl Open Source Ground Control Station

 (c) 2009 - 2014 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>

 This file is part of the QGROUNDCONTROL project

 QGROUNDCONTROL is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 QGROUNDCONTROL is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with QGROUNDCONTROL. If not, see <http://www.gnu.org/licenses/>.

 ======================================================================*/

#include <cmath>
#include <limits>

#include "QGCGeo.h"

// These defines are private
#define M_DEG_TO_RAD (M_PI / 180.0)

#define M_RAD_TO_DEG (180.0 / M_PI)

#define RADIUS_OF_EARTH			6371000			/* meters (m)		*/

static const float epsilon = std::numeric_limits<double>::epsilon();

void convertGeoToNed(QGeoCoordinate coord, QGeoCoordinate origin, double* n, double* e, double* d) {

    double lat_rad = coord.latitude() * M_DEG_TO_RAD;
    double lon_rad = coord.longitude() * M_DEG_TO_RAD;

    double ref_lon_rad = origin.longitude() * M_DEG_TO_RAD;
    double ref_lat_rad = origin.latitude() * M_DEG_TO_RAD;

    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double cos_d_lon = cos(lon_rad - ref_lon_rad);

    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double c = acos(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon);
    double k = (fabs(c) < epsilon) ? 1.0 : (c / sin(c));

    *n = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * RADIUS_OF_EARTH;
    *e = k * cos_lat * sin(lon_rad - ref_lon_rad) * RADIUS_OF_EARTH;

    *d = -(coord.altitude() - origin.altitude());
}

void convertNedToGeo(double n, double e, double d, QGeoCoordinate origin, QGeoCoordinate *coord) {
    double n_rad = n / RADIUS_OF_EARTH;
    double e_rad = e / RADIUS_OF_EARTH;
    double c = sqrtf(n_rad * n_rad + e_rad * e_rad);
    double sin_c = sin(c);
    double cos_c = cos(c);

    double ref_lon_rad = origin.longitude() * M_DEG_TO_RAD;
    double ref_lat_rad = origin.latitude() * M_DEG_TO_RAD;

    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double lat_rad;
    double lon_rad;

    if (fabs(c) > epsilon) {
        lat_rad = asin(cos_c * ref_sin_lat + (n_rad * sin_c * ref_cos_lat) / c);
        lon_rad = (ref_lon_rad + atan2(e_rad * sin_c, c * ref_cos_lat * cos_c - n_rad * ref_sin_lat * sin_c));

    } else {
        lat_rad = ref_lat_rad;
        lon_rad = ref_lon_rad;
    }

    coord->setLatitude(lat_rad * M_RAD_TO_DEG);
    coord->setLongitude(lon_rad * M_RAD_TO_DEG);

    coord->setAltitude(-d + origin.altitude());
}

