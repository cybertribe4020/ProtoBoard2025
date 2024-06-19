package frc.robot.util;

public class Interpolate {

  /**
   * Performs a piecewise linear interpolation of Y curves as functions of x, with curves defined in
   * an array with elements {x, Y1, ..., Yn} If x < minimum x, Y values corresponding to minimum x
   * are returned If x > maximum x, Y values corresponding to maximum x are returned
   *
   * @param x double value to interpolate with (e.g. x axis)
   * @param array double array of {x, Y1, ..., Yn} to interpolate over - x values must be increasing
   * @return double array [0] = x value, [1] = interpolated Y1 value, [n] = interpolated Yn value
   */
  public static double[] interpolate(double x, double[][] array) {

    int numYValues = array[0].length; // the number of Y curves in the array
    double[] result = new double[numYValues];

    result[0] = x; // return the x value used to interpolate

    // if out of range on the low side or only 1 array point, return first array values
    if ((x <= array[0][0]) || (array.length < 2)) {
      for (int i = 1; i < numYValues; i++) result[i] = array[0][i];
      return result;
    }

    int lastNdx = array.length - 1; // the number of points in the array

    // if out of range on high side, return last array values
    if (x >= array[lastNdx][0]) {
      for (int i = 1; i < numYValues; i++) result[i] = array[lastNdx][i];
      return result;
    }

    // loop through each of the Y curves to interpolate
    for (int j = 1; j < numYValues; j++) {

      // loop through points and find range to interpolate in
      for (int i = 1; i <= lastNdx; i++) {

        // y = mx + b, b = y[i] - mx[i]
        if (x < array[i][0]) {
          double slope = (array[i][j] - array[i - 1][j]) / (array[i][0] - array[i - 1][0]);
          result[j] = slope * x + (array[i][j] - slope * array[i][0]);
          break;
        }
      }
    }

    return result;
  }
}
