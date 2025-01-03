import 'package:flutter/material.dart';
import 'package:sensors_plus/sensors_plus.dart';
import 'package:vector_math/vector_math_64.dart' as vector_math;
import 'imu_tracker.dart';
import 'dart:async';
import 'dart:developer' as dev;
import 'package:permission_handler/permission_handler.dart';
import 'package:google_maps_flutter/google_maps_flutter.dart';

class IMUTrackingScreen extends StatefulWidget {
  const IMUTrackingScreen({super.key});

  @override
  State<IMUTrackingScreen> createState() => IMUTrackingScreenState();
}

class IMUTrackingScreenState extends State<IMUTrackingScreen> {
  final IMUTracker tracker = IMUTracker();
  StreamSubscription<GyroscopeEvent>? _gyroscopeSubscription;
  StreamSubscription<UserAccelerometerEvent>? _accelSubscription;
  StreamSubscription<MagnetometerEvent>? _magSubscription;
  bool _sensorsAvailable = false;
  static const Duration _sensorUpdateInterval = Duration(milliseconds: 20);
  DateTime? _lastGyroUpdate;
  DateTime? _lastAccelUpdate;
  DateTime? _lastMagUpdate;
  GoogleMapController? _mapController;
  final Set<Polyline> _pathPolylines = {};
  final List<LatLng> _pathPoints = [];
  static const LatLng _centerPosition = LatLng(0, 0);
  static const int _maxPathPoints = 1000;

  @override
  void initState() {
    super.initState();
    _requestPermissions();
  }

  Future<void> _requestPermissions() async {
    try {
      final locationStatus = await Permission.location.request();
      final activityStatus = await Permission.activityRecognition.request();

      if (locationStatus.isGranted && activityStatus.isGranted) {
        _checkPlatformSupport();
      } else {
        setState(() {
          _sensorsAvailable = false;
        });
        if (mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(
              content: Text('Location and sensor permissions are required'),
              duration: Duration(seconds: 3),
            ),
          );
        }
      }
    } catch (e) {
      dev.log('Error requesting permissions: $e');
    }
  }

  void _checkPlatformSupport() async {
    try {
      dev.log('Attempting to access gyroscope stream...');
      await SensorsPlatform.instance.gyroscopeEventStream().first;
      dev.log('Gyroscope stream accessed successfully');

      _sensorsAvailable = true;
      if (mounted) {
        setState(() {});
      }
      WidgetsBinding.instance.addPostFrameCallback((_) {
        dev.log('Initializing sensors...');
        _initializeSensors();
      });
    } catch (e, stackTrace) {
      dev.log('Sensors not available: $e\n$stackTrace');
      _sensorsAvailable = false;
      if (mounted) {
        setState(() {});
      }
    }
  }

  void _initializeSensors() {
    if (!_sensorsAvailable) {
      dev.log('Sensors not available, skipping initialization');
      return;
    }

    try {
      dev.log('Setting up gyroscope subscription...');
      _gyroscopeSubscription =
          SensorsPlatform.instance.gyroscopeEventStream().listen(
        (GyroscopeEvent event) {
          try {
            final now = DateTime.now();
            if (_lastGyroUpdate != null &&
                now.difference(_lastGyroUpdate!) < _sensorUpdateInterval) {
              return;
            }
            _lastGyroUpdate = now;

            if (event.x.isFinite && event.y.isFinite && event.z.isFinite) {
              tracker.processSensorData(event);
              setState(() {});
            }
          } catch (error) {
            dev.log("Error processing gyroscope data: $error");
          }
        },
        onError: (error) => dev.log('Gyroscope stream error: $error'),
      );

      dev.log('Setting up accelerometer subscription...');
      _accelSubscription =
          SensorsPlatform.instance.userAccelerometerEventStream().listen(
        (UserAccelerometerEvent event) {
          try {
            final now = DateTime.now();
            if (_lastAccelUpdate != null &&
                now.difference(_lastAccelUpdate!) < _sensorUpdateInterval) {
              return;
            }
            _lastAccelUpdate = now;

            if (event.x.isFinite && event.y.isFinite && event.z.isFinite) {
              tracker.processSensorData(event);
              setState(() {});
              _processAccelerometer(
                  event, now.millisecondsSinceEpoch.toDouble());
            }
          } catch (error) {
            dev.log("Error processing accelerometer data: $error");
          }
        },
        onError: (error) => dev.log('Accelerometer stream error: $error'),
      );

      dev.log('Setting up magnetometer subscription...');
      _magSubscription =
          SensorsPlatform.instance.magnetometerEventStream().listen(
        (MagnetometerEvent event) {
          try {
            final now = DateTime.now();
            if (_lastMagUpdate != null &&
                now.difference(_lastMagUpdate!) < _sensorUpdateInterval) {
              return;
            }
            _lastMagUpdate = now;

            if (event.x.isFinite && event.y.isFinite && event.z.isFinite) {
              tracker.processSensorData(event);
              setState(() {});
            }
          } catch (error) {
            dev.log("Error processing magnetometer data: $error");
          }
        },
        onError: (error) => dev.log('Magnetometer stream error: $error'),
      );

      dev.log('All sensor subscriptions set up successfully');
    } catch (e, stackTrace) {
      dev.log('Error initializing sensors: $e\n$stackTrace');
      _sensorsAvailable = false;
      if (mounted) {
        setState(() {});
      }
    }
  }

  void _processAccelerometer(UserAccelerometerEvent event, double currentTime) {
    try {
      if (!tracker.isCalibrated) return;

      const alpha = 0.8;
      final filteredX = event.x * alpha + (1 - alpha) * tracker.velocity.x;
      final filteredY = event.y * alpha + (1 - alpha) * tracker.velocity.y;

      tracker.velocity.x = filteredX;
      tracker.velocity.y = filteredY;

      _updateMapPosition();
    } catch (e) {
      dev.log('Error processing accelerometer: $e');
      _resetStates();
    }
  }

  @override
  void dispose() {
    _gyroscopeSubscription?.cancel();

    _accelSubscription?.cancel();
    _magSubscription?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text("Indoor Navigation"),
        actions: [
          if (_sensorsAvailable)
            IconButton(
              icon: const Icon(Icons.refresh),
              onPressed: () {
                tracker.recalibrate();
                ScaffoldMessenger.of(context).showSnackBar(
                  const SnackBar(content: Text('Recalibrating sensors...')),
                );
              },
            ),
        ],
      ),
      body: _sensorsAvailable
          ? _buildTrackingUI()
          : const Center(
              child: Text(
                'Sensors not available.\nPlease run on a device with motion sensors.',
                textAlign: TextAlign.center,
                style: TextStyle(fontSize: 16),
              ),
            ),
    );
  }

  Widget _buildTrackingUI() {
    return Column(
      children: [
        Expanded(
          flex: 3,
          child: Stack(
            children: [
              _buildMap(),
              Positioned(
                top: 16,
                right: 16,
                child: Column(
                  children: [
                    FloatingActionButton(
                      mini: true,
                      onPressed: () => _mapController?.animateCamera(
                        CameraUpdate.zoomIn(),
                      ),
                      child: const Icon(Icons.add),
                    ),
                    const SizedBox(height: 8),
                    FloatingActionButton(
                      mini: true,
                      onPressed: () => _mapController?.animateCamera(
                        CameraUpdate.zoomOut(),
                      ),
                      child: const Icon(Icons.remove),
                    ),
                    const SizedBox(height: 8),
                    FloatingActionButton(
                      mini: true,
                      onPressed: _resetStates,
                      child: const Icon(Icons.clear),
                    ),
                  ],
                ),
              ),
            ],
          ),
        ),
        Expanded(
          flex: 2,
          child: Container(
            padding: const EdgeInsets.all(16),
            child: Card(
              elevation: 4,
              child: Padding(
                padding: const EdgeInsets.all(16),
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.spaceAround,
                  children: [
                    _buildInfoRow(
                      Icons.directions_walk,
                      "Steps",
                      tracker.stepCount.toString(),
                    ),
                    _buildInfoRow(
                      Icons.location_on,
                      "Position",
                      _formatVector(tracker.position),
                    ),
                    _buildInfoRow(
                      Icons.speed,
                      "Velocity",
                      _formatVector(tracker.velocity),
                    ),
                    _buildInfoRow(
                      Icons.rotate_right,
                      "Orientation",
                      _formatVector(tracker.orientation),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildInfoRow(IconData icon, String label, String value) {
    return Container(
      padding: const EdgeInsets.symmetric(vertical: 8),
      decoration: BoxDecoration(
        border: Border(bottom: BorderSide(color: Colors.grey.withAlpha(51))),
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Row(
            children: [
              Icon(icon, color: Theme.of(context).primaryColor),
              const SizedBox(width: 8),
              Text(label, style: const TextStyle(fontWeight: FontWeight.bold)),
            ],
          ),
          Text(
            value,
            style: TextStyle(color: Theme.of(context).primaryColor),
          ),
        ],
      ),
    );
  }

  String _formatVector(vector_math.Vector3 vector) {
    return "(${vector.x.toStringAsFixed(2)}, "
        "${vector.y.toStringAsFixed(2)}, "
        "${vector.z.toStringAsFixed(2)})";
  }

  Widget _buildMap() {
    return GoogleMap(
      initialCameraPosition: const CameraPosition(
        target: _centerPosition,
        zoom: 18,
      ),
      polylines: _pathPolylines,
      onMapCreated: (GoogleMapController controller) {
        _mapController = controller;
      },
    );
  }

  void _updateMapPosition() {
    if (_mapController == null) return;

    final newPosition = LatLng(
      _centerPosition.latitude + (tracker.position.x * 0.00001),
      _centerPosition.longitude + (tracker.position.y * 0.00001),
    );

    if (_pathPoints.length > _maxPathPoints) {
      _pathPoints.removeRange(0, _pathPoints.length - _maxPathPoints);
    }

    _pathPoints.add(newPosition);
    _updatePolyline();

    if (_pathPoints.length > 1) {
      final lastPoint = _pathPoints[_pathPoints.length - 2];
      if (_calculateDistance(lastPoint, newPosition) > 0.00001) {
        _mapController!.animateCamera(CameraUpdate.newLatLng(newPosition));
      }
    }
  }

  double _calculateDistance(LatLng start, LatLng end) {
    return ((start.latitude - end.latitude).abs() +
            (start.longitude - end.longitude).abs()) /
        2;
  }

  void _updatePolyline() {
    _pathPolylines.clear();
    _pathPolylines.add(
      Polyline(
        polylineId: const PolylineId('path'),
        points: _pathPoints,
        color: Colors.blue,
        width: 3,
        patterns: [PatternItem.dash(20), PatternItem.gap(10)],
      ),
    );
    setState(() {});
  }

  void _resetStates() {
    _pathPoints.clear();
    _pathPolylines.clear();
    setState(() {});
  }
}

class TrackingPainter extends CustomPainter {
  final vector_math.Vector3 position;
  final Paint _pathPaint;
  final Paint _positionPaint;
  final List<Offset> _pathPoints = [];

  TrackingPainter(this.position)
      : _pathPaint = Paint()
          ..color = Colors.blue.withAlpha(128)
          ..strokeWidth = 2
          ..style = PaintingStyle.stroke,
        _positionPaint = Paint()
          ..color = Colors.red
          ..strokeWidth = 8
          ..strokeCap = StrokeCap.round;

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final scale = size.width / 10;

    // Convert position to screen coordinates
    final currentPos = Offset(
      center.dx + position.x * scale,
      center.dy + position.y * scale,
    );

    // Add point to path
    _pathPoints.add(currentPos);
    if (_pathPoints.length > 100) {
      _pathPoints.removeAt(0);
    }

    // Draw grid
    _drawGrid(canvas, size, scale);

    // Draw path
    if (_pathPoints.length > 1) {
      final path = Path()..moveTo(_pathPoints[0].dx, _pathPoints[0].dy);
      for (int i = 1; i < _pathPoints.length; i++) {
        path.lineTo(_pathPoints[i].dx, _pathPoints[i].dy);
      }
      canvas.drawPath(path, _pathPaint);
    }

    // Draw current position
    canvas.drawCircle(currentPos, 5, _positionPaint);
  }

  void _drawGrid(Canvas canvas, Size size, double scale) {
    final gridPaint = Paint()
      ..color = Colors.grey.withAlpha(77)
      ..strokeWidth = 1;

    // Draw vertical lines
    for (double x = 0; x <= size.width; x += scale) {
      canvas.drawLine(
        Offset(x, 0),
        Offset(x, size.height),
        gridPaint,
      );
    }

    // Draw horizontal lines
    for (double y = 0; y <= size.height; y += scale) {
      canvas.drawLine(
        Offset(0, y),
        Offset(size.width, y),
        gridPaint,
      );
    }
  }

  @override
  bool shouldRepaint(TrackingPainter oldDelegate) {
    return position != oldDelegate.position;
  }
}
