//
// Created by Stanislav Olekhnovich on 13/10/2017.
// see https://github.com/fspirit/path-planning-starter
// Modified by Anthony M Knight 30/01/2018
//

#ifndef PATH_PLANNING_CARTESIANPOINT_H
#define PATH_PLANNING_CARTESIANPOINT_H


struct CartesianPoint
{
    double X;
    double Y;
    double ThetaRads;  //Renamed to emphasize this is radians

    CartesianPoint() = default;
    CartesianPoint(double X, double Y, double ThetaRads = 0.0) : X(X), Y(Y), ThetaRads(ThetaRads) {}
    inline CartesianPoint ToLocal(const CartesianPoint &localReferencePoint) const
    {
        double shiftX = X - localReferencePoint.X;
        double shiftY = Y - localReferencePoint.Y;
        return {(shiftX * cos(-localReferencePoint.ThetaRads) - shiftY * sin(-localReferencePoint.ThetaRads)),
                (shiftX * sin(-localReferencePoint.ThetaRads) + shiftY * cos(-localReferencePoint.ThetaRads))};
    };
    inline CartesianPoint ToGlobal(const CartesianPoint& localReferencePoint) const
    {
        return { localReferencePoint.X + (X * cos(localReferencePoint.ThetaRads) - Y * sin(localReferencePoint.ThetaRads)),
                 localReferencePoint.Y + (X * sin(localReferencePoint.ThetaRads) + Y * cos(localReferencePoint.ThetaRads))};
    };
	inline double EuclidDistance(const CartesianPoint& p1) const
	{
		return sqrt((X - p1.X)*(X - p1.X) + (Y - p1.Y)*(Y - p1.Y));
	}

};


#endif //PATH_PLANNING_CARTESIANPOINT_H
