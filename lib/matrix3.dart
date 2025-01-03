class KalmanMatrix {
  final List<List<double>> data;

  KalmanMatrix.fromList(List<num> flatList)
      : assert(flatList.length == 9, 'List must contain exactly 9 elements'),
        data = [
          [flatList[0], flatList[1], flatList[2]],
          [flatList[3], flatList[4], flatList[5]],
          [flatList[6], flatList[7], flatList[8]]
        ].map((row) => row.map((v) => v.toDouble()).toList()).toList();

  KalmanMatrix.from2DList(List<List<double>> list)
      : assert(list.length == 3 && list.every((row) => row.length == 3),
            '2D list must be 3x3'),
        data=list;

  KalmanMatrix.identity(int size)
      : assert(size == 3, 'Only 3x3 matrices are supported'),
        data = List<List<double>>.generate(size,
            (i) => List<double>.generate(size, (j) => i == j ? 1.0 : 0.0));

  KalmanMatrix operator *(KalmanMatrix other) {
    final result = List<List<double>>.generate(
        3, (i) => List<double>.generate(3, (j) => 0.0));
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
          result[i][j] += data[i][k] * other.data[k][j];
        }
      }
    }
    return KalmanMatrix.from2DList(result);
  }

  KalmanMatrix operator +(KalmanMatrix other) {
    final result = List<List<double>>.generate(3,
        (i) => List<double>.generate(3, (j) => data[i][j] + other.data[i][j]));
    return KalmanMatrix.from2DList(result);
  }

  KalmanMatrix operator -(KalmanMatrix other) {
    final result = List<List<double>>.generate(3,
        (i) => List<double>.generate(3, (j) => data[i][j] - other.data[i][j]));
    return KalmanMatrix.from2DList(result);
  }

  KalmanMatrix transpose() {
    final result = List<List<double>>.generate(
        3, (i) => List<double>.generate(3, (j) => data[j][i]));
    return KalmanMatrix.from2DList(result);
  }

  KalmanMatrix inverse() {
    double det = _determinant();
    if (det.abs() < 1e-10) {
      throw Exception(
          'Matrix is not invertible (determinant too close to zero)');
    }

    List<List<double>> adj = _adjugate();
    final result = List<List<double>>.generate(
        3, (i) => List<double>.generate(3, (j) => adj[i][j] / det));
    return KalmanMatrix.from2DList(result);
  }

  double _determinant() {
    return data[0][0] * (data[1][1] * data[2][2] - data[1][2] * data[2][1]) -
        data[0][1] * (data[1][0] * data[2][2] - data[1][2] * data[2][0]) +
        data[0][2] * (data[1][0] * data[2][1] - data[1][1] * data[2][0]);
  }

  List<List<double>> _adjugate() {
    List<List<double>> adj = List.generate(3, (_) => List.filled(3, 0.0));
    adj[0][0] = data[1][1] * data[2][2] - data[1][2] * data[2][1];
    adj[0][1] = -(data[0][1] * data[2][2] - data[0][2] * data[2][1]);
    adj[0][2] = data[0][1] * data[1][2] - data[0][2] * data[1][1];
    adj[1][0] = -(data[1][0] * data[2][2] - data[1][2] * data[2][0]);
    adj[1][1] = data[0][0] * data[2][2] - data[0][2] * data[2][0];
    adj[1][2] = -(data[0][0] * data[1][2] - data[0][2] * data[1][0]);
    adj[2][0] = data[1][0] * data[2][1] - data[1][1] * data[2][0];
    adj[2][1] = -(data[0][0] * data[2][1] - data[0][1] * data[2][0]);
    adj[2][2] = data[0][0] * data[1][1] - data[0][1] * data[1][0];
    return adj;
  }

  double get(int i, int j) => data[i][j];
  void setEntry(int i, int j, double value) => data[i][j] = value;

  @override
  String toString() {
    return data.map((row) => row.join(', ')).join('\n');
  }
}
