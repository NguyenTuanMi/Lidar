package frc.robot;
import static frc.robot.Constants.LIDAR_DATA.*;

public class MathFunction {
    public static double getLamwbertWFunction(double z) {
        double S = 0.0;
      for (int n=1; n <= 1000; n++)
      {
          double Se = S * StrictMath.pow(StrictMath.E, S);
          double S1e = (S+1) * 
              StrictMath.pow(StrictMath.E, S);
          if (offset > StrictMath.abs((z-Se)/S1e))
          {
              return S;
          }
          S -= 
              (Se-z) / (S1e - (S+2) * (Se-z) / (2*S+2));
      }
      return S;
    }

    // calculate the value at x of gln(x) + x - b
    public static double function_value(double g, double b, double x ) {
        if (x>0){
            return g*Math.log(x) + x - b;
        }
        else {
            System.out.print("Unsolvable x");
            return 0;
        }
    }

    // calculate the derivative at x of gln(x) + x - b
    public static double derivative_function_value(double g, double x) {
        if (x <= 0) {
            System.out.print("Unsolvable x");
            return 0;
        }
        else {
            return g/x + 1;
        }
    }
    // solving the equation gln(x) + x = b using Newton - Raphson method
    public static double solving_equation(double g, double x, double b) {
        if(Math.abs(function_value(g, b, x)/derivative_function_value(g, x)) < 0.01 ) {
            return x - function_value(g, b, x)/derivative_function_value(g, x);
        }
        else {
            double x_sec = x - function_value(g, b, x)/derivative_function_value(g, x);
            return solving_equation(g, x_sec, b);
        }
    }
}
