// Copyright 2021 University of Washington Applied Physics Laboratory
//
// An implementation of an AbstractSonarInterface which
// wraps a ROS acoustic_msgs::SonarImage
//

#include "sonar_image_proc/AbstractSonarInterface.h"
#include "sonar_image_proc/ColorMaps.h"

namespace nps_uw_multibeam_sonar {

typedef std::complex<float> Complex;
typedef std::valarray<Complex> CArray;
typedef std::valarray<CArray> CArray2D;

using namespace std;
using namespace cv;

struct NPSSimulatedSonarInterface : public sonar_image_proc::AbstractSonarInterface {
    explicit NPSSimulatedSonarInterface(const CArray2D &beams,
            const vector<float> &azimuths,
            const vector<float> &ranges,
            float sensor_gain )
      : _beams(beams),
        _azimuths(azimuths),
        _ranges(ranges),
        _sensorGain( sensor_gain ) {;}

    const std::vector<float> &ranges() const override   { return _ranges; }
    const std::vector<float> &azimuths() const override { return _azimuths; }

    float intensity_float(int b, int r) const override {
        // \todo As of right now, the sonar image stored in _beams is
        // flipped L-to-R. This is a confusing inconsistency
        const size_t bb = nAzimuth() - b - 1;
        return abs(_beams[bb][r]);
    }

    const CArray2D &_beams;
    const vector<float> &_azimuths, &_ranges;
    float _sensorGain;
};

struct NPSGreyscaleColorMap : public sonar_image_proc::SonarColorMap {
    NPSGreyscaleColorMap(float plotScaler)
        : _plotScaler(plotScaler)
    {;}

    float lookup_float(const sonar_image_proc::AbstractSonarInterface &ping,
                                int bearing_idx, int range_idx) const override {
       const auto intensity = ping.intensity_float(bearing_idx, range_idx);

       // Perform the actual color mapping from the intensity to a greyscale image
       return intensity/2500.0*_plotScaler;
    }

    float _plotScaler;
};

}  // namespace draw_sonar