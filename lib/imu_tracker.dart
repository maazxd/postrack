import 'package:sensors_plus/sensors_plus.dart';
import 'package:vector_math/vector_math_64.dart';
import 'dart:math' as math;
import 'matrix3.dart';

class IMUTracker {
  // Public properties
  int stepCount = 0;
  Vector3 position = Vector3.zero();
  Vector3 velocity = Vector3.zero();
  Vector3 orientation = Vector3.zero();

  // Sensor calibration values
  Vector3 _accelBias = Vector3.zero();
  final Vector3 _gyroBias = Vector3.zero();
  bool _isCalibrated = false;

  // Constants for step detection
  static const double _stepThreshold = 10.0;
  static const double _timeThreshold = 250.0;
  static const int _calibrationSamples = 100;
  static const double _lowPassAlpha = 0.1;
  static const double _gravityMagnitude = 9.81;

  // State variables
  double _lastStepTime = 0;
  Vector3 _filteredAccel = Vector3.zero();
  final List<Vector3> _calibrationAccelData = [];
  final List<Vector3> _calibrationGyroData = [];

  // Error margins
  static const double _maxAcceleration = 50.0; // m/s²
  static const double _maxAngularVelocity = 20.0; // rad/s

  // Kalman filter matrices
  late KalmanMatrix _stateMatrix;
  late KalmanMatrix _errorCovariance;
  final KalmanMatrix _processNoise;
  final KalmanMatrix _measurementNoise;
  final KalmanMatrix _transitionMatrix;
  final KalmanMatrix _measurementMatrix;

  IMUTracker()
      : _processNoise = KalmanMatrix.fromList(
            [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]),
        _measurementNoise = KalmanMatrix.fromList(
            [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]),
        _transitionMatrix = KalmanMatrix.fromList(
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
        _measurementMatrix = KalmanMatrix.fromList(
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]) {
    _initializeKalmanFilter();
  }

  void _initializeKalmanFilter() {
    _stateMatrix =
        KalmanMatrix.fromList([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

    _errorCovariance =
        KalmanMatrix.fromList([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
  }

  Vector3 _applyKalmanFilter(Vector3 measurement, double dt) {
    try {
      // Update transition matrix
      _transitionMatrix.setEntry(0, 1, dt);
      _transitionMatrix.setEntry(0, 2, 0.5 * dt * dt);
      _transitionMatrix.setEntry(1, 2, dt);

      // Predict step
      _stateMatrix = _transitionMatrix * _stateMatrix;

      _errorCovariance =
          _transitionMatrix * _errorCovariance * _transitionMatrix.transpose() +
              _processNoise;

      // Measurement update
      final measurementMatrix = KalmanMatrix.fromList([
        measurement.x,
        0.0,
        0.0,
        0.0,
        measurement.y,
        0.0,
        0.0,
        0.0,
        measurement.z
      ]);

      final innovation =
          measurementMatrix - (_measurementMatrix * _stateMatrix);

      final kalmanGain = _calculateKalmanGain();

      // Update state
      _stateMatrix = _stateMatrix + (kalmanGain * innovation);

      return Vector3(_stateMatrix.get(0, 0), _stateMatrix.get(1, 0),
          _stateMatrix.get(2, 0));
    } catch (e) {
      rethrow;
    }
  }

  KalmanMatrix _calculateKalmanGain() {
    try {
      final innovationCovariance = (_measurementMatrix *
              _errorCovariance *
              _measurementMatrix.transpose()) +
          _measurementNoise;

      final gain = _errorCovariance *
          _measurementMatrix.transpose() *
          innovationCovariance.inverse();
      return gain;
    } catch (e) {
      rethrow;
    }
  }

  void processSensorData(dynamic event) {
    try {
      final currentTime = DateTime.now().millisecondsSinceEpoch.toDouble();

      if (!_isCalibrated && event is UserAccelerometerEvent) {
        _collectCalibrationData(event);
        return;
      }

      if (event is GyroscopeEvent) {
        _processGyroscope(event);
      } else if (event is UserAccelerometerEvent) {
        _processAccelerometer(event, currentTime);
      } else if (event is MagnetometerEvent) {
        _processMagnetometer(event);
      }
    } catch (e) {
      _resetStates();
    }
  }

  void _processGyroscope(GyroscopeEvent event) {
    var gyroData = Vector3(event.x, event.y, event.z);

    // Apply bias correction and threshold
    gyroData -= _gyroBias;
    if (gyroData.length > _maxAngularVelocity) {
      return;
    }

    const dt = 0.01;
    orientation += gyroData * dt;
    _normalizeAngles();
  }

  void _processAccelerometer(UserAccelerometerEvent event, double currentTime) {
    try {
      var accel = Vector3(event.x, event.y, event.z);

      accel -= _accelBias;

      if (accel.length > _maxAcceleration) {
        return;
      }

      const dt = 0.01;
      final filteredAccel = _applyKalmanFilter(accel, dt);

      _filteredAccel = _lowPassFilter(_filteredAccel, filteredAccel);

      _detectStep(_filteredAccel, currentTime);
      _updatePosition(_filteredAccel);
    } catch (e) {
      _resetStates();
    }
  }

  void _processMagnetometer(MagnetometerEvent event) {}

  void _detectStep(Vector3 accel, double currentTime) {
    final magnitude = accel.length;
    final timeDelta = currentTime - _lastStepTime;

    if (magnitude > _stepThreshold &&
        timeDelta > _timeThreshold &&
        _isValidStepPattern(magnitude)) {
      stepCount++;
      _lastStepTime = currentTime;
    }
  }

  bool _isValidStepPattern(double magnitude) {
    // Add pattern recognition to reduce false positives
    return magnitude > _stepThreshold && magnitude < _stepThreshold * 2.0;
  }

  void _updatePosition(Vector3 accel) {
    const dt = 0.01;

    // Remove gravity component
    final gravityCompensated = _removeGravity(accel);

    // Update velocity with drift compensation
    velocity += gravityCompensated * dt;
    _applyVelocityDriftCompensation();

    // Update position
    position += velocity * dt;
  }

  Vector3 _removeGravity(Vector3 accel) {
    // Implement gravity compensation based on orientation
    return accel - Vector3(0, 0, _gravityMagnitude);
  }

  void _applyVelocityDriftCompensation() {
    // Apply damping to reduce velocity drift
    const dampingFactor = 0.98;
    velocity *= dampingFactor;

    // Zero out very small velocities to prevent drift
    if (velocity.length < 0.01) {
      velocity = Vector3.zero();
    }
  }

  Vector3 _lowPassFilter(Vector3 prev, Vector3 current) {
    return prev * (1 - _lowPassAlpha) + current * _lowPassAlpha;
  }

  void _normalizeAngles() {
    // Keep angles between -π and π
    orientation = Vector3(
      _normalizeAngle(orientation.x),
      _normalizeAngle(orientation.y),
      _normalizeAngle(orientation.z),
    );
  }

  double _normalizeAngle(double angle) {
    while (angle > math.pi) {
      angle -= 2 * math.pi;
    }
    while (angle < -math.pi) {
      angle += 2 * math.pi;
    }
    return angle;
  }

  void _collectCalibrationData(UserAccelerometerEvent event) {
    final accel = Vector3(event.x, event.y, event.z);
    _calibrationAccelData.add(accel);

    if (_calibrationAccelData.length >= _calibrationSamples) {
      _calculateBiases();
      _isCalibrated = true;
    }
  }

  void _calculateBiases() {
    // Calculate average of calibration samples for bias
    _accelBias = _calibrationAccelData.reduce((a, b) => a + b) /
        _calibrationAccelData.length.toDouble();

    // Remove gravity from acceleration bias
    _accelBias.z -= _gravityMagnitude;

    // Clear calibration data
    _calibrationAccelData.clear();
  }

  void _resetStates() {
    velocity = Vector3.zero();
    _filteredAccel = Vector3.zero();
  }

  // Public method to force recalibration
  void recalibrate() {
    _isCalibrated = false;
    _calibrationAccelData.clear();
    _calibrationGyroData.clear();
    _resetStates();
  }

  bool get isCalibrated => _isCalibrated;
}
