import os
from ubitrack.core import math as utmath, measurement, util
import numpy as np
import math
import tempfile


def test_basic_datatypes():
    "test basic data types: Vector2-8, Matrix33-44, Pose, Quaternion"

    with tempfile.TemporaryDirectory() as tmpdirname:
        tmpfile = os.path.join(tmpdirname, "test.calib")
        ts = measurement.now()

        p = measurement.Position(ts, utmath.Vector3d(1.0, 2.0, 3.0))
        util.writeCalibMeasurementPosition(tmpfile, p)
        pr = util.readCalibMeasurementPosition(tmpfile)
        assert np.all(p.get() == pr)

        p = measurement.Rotation(ts, utmath.Quaternion())
        util.writeCalibMeasurementRotation(tmpfile, p)
        pr = util.readCalibMeasurementRotation(tmpfile)
        assert p.get() == pr


def test_positionrecord():
    "test reading a position3d recording"

    fname = os.path.join(os.path.dirname(__file__), "position_record_test.log")
    st = util.PositionStreamReader(fname)
    v = st.values()
    for i in v:
        print(i)
        p = i.get()
        print(p)
        print(i.time())
