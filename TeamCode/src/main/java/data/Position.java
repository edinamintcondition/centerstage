package data;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

@SuppressLint("DefaultLocale")
public final class Position {
    public double x, y, dx, dy, a;

    public Position(double x, double y, double dx, double dy, double a) {
        this.x = x;
        this.y = y;
        this.a = a;
        this.dx = dx;
        this.dy = dy;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format("%.2f,%.2f @%.2f", x, y, a);
    }

    public double dist(Position p) {
        double x = this.x - p.x;
        double y = this.y - p.y;
        return Math.sqrt(x * x + y * y);
    }

    public Point toRobotRel(Point p) {
        double tx = p.x - this.x;
        double ty = p.y - this.y;
        return new Point(
                (tx * dy) - (ty * dx),
                (tx * dx) + (ty * dy));
    }

    public Position addRobotRel(Point p) {
        double tx = dy * p.x + dx * p.y;
        double ty = -dx * p.x + dy * p.y;
        return new Position(x + tx, y + ty, dx, dy, a);
    }
}
