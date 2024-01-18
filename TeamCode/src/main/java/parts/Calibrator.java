package parts;

public class Calibrator {
    private final double goal, tol;
    private double minX, maxX, fMin, fMax, guess;
    private int iter;

    public Calibrator(double goal, double initGuess) {
        this(goal, initGuess, 1e-6);
    }

    public Calibrator(double goal, double initGuess, double tol) {
        this.goal = goal;
        this.tol = tol;

        double dx = 0.1 * Math.abs(initGuess);
        if (dx == 0)
            dx = Math.abs(tol);

        minX = initGuess - dx;
        maxX = initGuess + dx;
        fMin = 9e99;
        fMax = 9e99;

        guess = minX;
    }

    public double getGuess() {
        return guess;
    }

    public void updateGuess(double y) {
        iter++;

        updateRange(y);
        updateGuess();
    }

    private void updateGuess() {
        if (iter == 1) {
            guess = maxX;
            return;
        }

        double dMin = errSign(fMin);
        double dMax = errSign(fMax);
        double dx = maxX - minX;
        if (dMin < 0 && dMax < 0) {
            if (fMin < fMax)
                guess = maxX + dx;
            else
                guess = minX - dx;
        } else if (dMin > 0 && dMax > 0) {
            if (fMin > fMax)
                guess = maxX + dx;
            else
                guess = minX - dx;
        } else {
            guess = (minX + maxX) / 2;
        }
    }

    private void updateRange(double y) {
        double x = guess;
        double dy = errSign(y);

        if (x <= minX) {
            updateMin(x, y);
        } else if (x >= maxX) {
            updateMax(x, y);
        } else {
            if (dy <= 0)
                updateMin(x, y);
            if (dy >= 0)
                updateMax(x, y);
        }
    }

    @Override
    public String toString() {
        return String.format("cal %d -> %f: f(%f) = %f .. f(%f) = %f, guess = %f",
                iter, goal, minX, fMin, maxX, fMax, guess);
    }

    private double errSign(double y) {
        double d = y - goal;
        if (Math.abs(y) < tol)
            return 0;
        else
            return Math.signum(d);
    }

    private void updateMin(double x, double y) {
        minX = x;
        fMin = y;
    }

    private void updateMax(double x, double y) {
        maxX = x;
        fMax = y;
    }
}

