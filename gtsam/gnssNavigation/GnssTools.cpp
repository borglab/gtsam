/**
 * @file   GnssTools.cpp
 * @brief  Tools required to process GNSS data -- (i.e. ECEF to ENU transformation)
 * @author Ryan Watson & Jason Gross
 */

#include <gtsam/gnssNavigation/GnssTools.h>

namespace gtsam {

Vector obsMap(const Point3 &p1, const Point3 &p2, const int &Trop) {
        /*
           inputs ::
           p1 --> ECEF xyz coordinates of satellite [meter]
           p2 --> ECEF xyz coordinates of receiver [meter]
           Trop --> Troposphere modeling switch
           outputs ::
           H --> measurement mapping
         */
        double r = sqrt( ((p1.x()-p2.x()))*(p1.x()-p2.x()) + ((p1.y()-p2.y())*(p1.y()-p2.y())) + ((p1.z()-p2.z())*(p1.z()-p2.z())) );
        if (Trop == 1) {
                double el = calcEl(p1,p2);
                double mapT = tropMap(el);
                Vector5 H; H << (p1.x()-p2.x())/r, (p1.y()- p2.y())/r,
                (p1.z()-p2.z())/r, 1.0,mapT;
                return H;
        }
        else {
                Vector5 H; H << (p1.x()-p2.x())/r, (p1.y()- p2.y())/r,
                (p1.z()-p2.z())/r, 1.0, 0.0;
                return H;
        }
}

Vector obsMapNED(const Point3 &p1, const Point3 &p2, const int &Trop) {
        /*
           inputs ::
           p1 --> ECEF xyz coordinates of satellite [meter]
           p2 --> ECEF xyz coordinates of receiver [meter]
           Trop --> Troposphere modeling switch
           outputs ::
           H --> measurement mapping
         */
        double r = sqrt( ((p1.x()-p2.x()))*(p1.x()-p2.x()) + ((p1.y()-p2.y())*(p1.y()-p2.y())) + ((p1.z()-p2.z())*(p1.z()-p2.z())) );
        if (Trop == 1) {
                double el = calcElNed(p1);
                double mapT = tropMap(el);
                Vector5 H; H << (p1.x()-p2.x())/r, (p1.y()- p2.y())/r,
                (p1.z()-p2.z())/r, 1.0,mapT;
                return H;
        }
        else {
                Vector5 H; H << (p1.x()-p2.x())/r, (p1.y()- p2.y())/r,
                (p1.z()-p2.z())/r, 1.0, 0.0;
                return H;
        }
}

Eigen::VectorXi getPRN(const Matrix &p) {
        /*
           inputs ::
           p --> GNSS data packed in pre-specified format
           outputs ::
           prnVec --> Vector of prn numbers corresponding to visible satellites at given epoch
         */

        int nSats = static_cast<int>(p(0));
        Eigen::VectorXi prnVec(nSats);
        int count = 2;
        for (int i=0; i<nSats; i++) {
                prnVec(i) = static_cast<int>(p(count));
                count = count+7;
        }
        return prnVec;
}

bool checkPRN(const Eigen::VectorXi &p, const int &n) {
        /*
           input ::
           p --> prn vector
           n --> prn value to check
           output ::
           true --> if n is present in current prn vec.
           false --> otherwise
         */

        for (int i=0; i<p.size(); i++) {
                if (p(i) == n ) {return true; }
        }
        return false;
}

Matrix earthToNavTrans( const Point3 &p1){
        /*
           inputs ::
            p1 --> ECEF xyz coordinates of the platform
           outputs ::
            R --> rotation matrix from Earth fram to Navigation frame
         */
        Vector3 llh=xyz2llh(p1);
        double sLat = sin(llh(0));
        double sLon = sin(llh(1));
        double cLat = cos(llh(0));
        double cLon = cos(llh(1));

        Matrix R = (Matrix(3,3) << -1*sLat*cLon, -1*sLon, -1*cLat*cLon,
                    -1*sLat*sLon, cLon, -1*cLat*sLon,
                    cLat, 0.0, -1*sLat ).finished();

        return R.inverse();

}

double deltaObs(const Point3 &p1, const Point3 &p2, const double &meas){
        /*
           inputs ::
           p1 --> ECEF xyz coordinates of satellite [meter]
           p2 --> ECEF xyz coordinates of receiver [meter]
           meas --> observed range between satellite and receiver [meter]
           outputs ::
           deltaR --> difference observed pseudorange and modeled pseudorange [meter]
         */
        double r = norm3(p1-p2) + tropMap(calcEl(p1,p2))*tropDry(p2);
        return meas - r;
}

double deltaTrop(const Point3 &p1, const Point3 &p2){
        /*
           inputs ::
           p1 --> ECEF xyz coordinates of satellite [meter]
           p2 --> ECEF xyz coordinates of receiver [meter]
           meas --> observed range between satellite and receiver [meter]
         */
        return tropMap(calcEl(p1,p2))*tropDry(p2);
}

Point3 xyz2llh(const Point3 &p1){
        /*
           inputs ::
           p1 --> ECEF xyz receiver coordinates [meter]
           output ::
           posLLH --> latitude, longitude, height [rad,rad,meter]
         */
        double x2= pow(p1.x(),2);
        double y2= pow(p1.y(),2);
        double z2= pow(p1.z(),2);

        double e= sqrt(1.0-((semiMinor/semiMajor)*(semiMinor/semiMajor)));
        double b2= pow(semiMinor,2);
        double e2= pow(e,2);
        double ep = e*(semiMajor/semiMinor);
        double r = sqrt(x2+y2);
        double r2 = pow(r,2);
        double E2 = pow(semiMajor,2)- pow(semiMinor,2);
        double F = 54*b2*z2;
        double G = r2 + (1-e2)*z2-e2*E2;
        double c = (pow(e2,2)*F*r2)/(pow(G,3));
        double s = pow((1.0 + c + sqrt(c*c +2*c )),1.0/3.0);

        double P = F /(3.0 * (s+1/s+1)*(s+1/s+1)* G*G);
        double Q = sqrt(1.0 + 2*e2*e2*P);
        double ro = -(P*e2*r)/(1+Q) + sqrt((semiMajor*semiMajor/2)*(1+1/Q)-(P*(1-e2)*z2)/(Q*(1+Q))-P*r2/2);
        double tmp = pow((r-e2*ro),2);
        double U = sqrt( tmp +z2 );
        double V = sqrt( tmp + (1-e2)*z2 );
        double zo = (b2*p1.z())/(semiMajor*V);

        double height = U*(1.0 - b2/(semiMajor*V));
        double lat  = std::atan(( p1.z() + ep*ep*zo)/r);
        double temp= std::atan(p1.y()/p1.x());
        double longitude;
        if(p1.x() >= 0.0) {
                longitude= temp;
        }else if(( p1.x() <0.0 )&&(p1.y()>=0.0)) {
                longitude = temp + M_PI;
        } else{
                longitude = temp - M_PI;

        }
        Point3 llh(lat,longitude,height);
        return llh;
}

Point3 inertialToECEF( const Point3& inertialPosition, const double t, const double t0){
        /*
         * inputs ::
         *  inertialPos -- > ECI position vector
         *  t --> current time [sec]
         *  t0 --> time when coordinate frames aligned [sec]
         * ouputs ::
         *  ecefPos --> ECEF position vector [meters]
         */
        double cT = cos(earthRot*(t-t0));
        double sT = sin(earthRot*(t-t0));
        Matrix Rie = (Matrix(3,3) << cT, sT, 0, -sT, cT, 0, 0, 0, 1 ).finished();
        Point3 ecefPosition= Rie*inertialPosition;
        return ecefPosition;
}

Point3 enu2xyz(const Point3& p1, const Point3& p2) {
        /*
           inputs ::
           p1 --> enu coordinates [meter]
           p2 --> ECEF xyz origin coordinates [meter]
           outputs ::
            posXYZ ---> ECEF XYZ position vector [meters]
         */
        Vector3 orgLLH = xyz2llh(p2);
        double sinPhi = sin(orgLLH(0));
        double cosPhi = cos(orgLLH(0));
        double sinLam = sin(orgLLH(1));
        double cosLam = cos(orgLLH(1));
        Matrix R = ( Matrix(3,3) << -1*sinLam, cosLam, 0,
                     -1*sinPhi*cosLam, -1*sinPhi*sinLam, cosPhi,
                     cosPhi*cosLam, cosPhi*sinLam, sinPhi ).finished();
        Point3 deltaXYZ;
        deltaXYZ = R.inverse()*p1;
        return p2 + deltaXYZ;
}

Point3 ned2enu(const Point3& p1) {
        Matrix enuConv = ( Matrix(3,3) << 0, 1, 0, 1, 0, 0, 0, 0, -1 ).finished();
        return enuConv*p1;
}

Point3 xyz2enu(const Point3 &p1, const Point3 &p2){
        /*
           inputs ::
           p1 --> ECEF xyz coordinates [meter]
           p2 --> ECEF xyz origin coordinates [meter]
              outputs ::
                  posENU --> ENU position coordinates [meters]
         */

        Vector3 posDiff = p1 - p2;
        Vector3 orgLLH = xyz2llh(p2);
        double sinPhi = sin(orgLLH(0));
        double cosPhi = cos(orgLLH(0));
        double sinLam = sin(orgLLH(1));
        double cosLam = cos(orgLLH(1));
        Matrix R = ( Matrix(3,3) << (-1*sinLam), cosLam, 0, ((-1*sinPhi)*cosLam), ((-1*sinPhi)*sinLam), cosPhi, (cosPhi*cosLam), (cosPhi*sinLam), sinPhi ).finished();
        Vector3 pos;
        pos = R*posDiff;
        Point3 posENU;
        return posENU = Point3(pos(0), pos(1), pos(2));
}

double calcElNed(const Point3& p1){
        /*
         * inputs ::
         *  p1 --> ECEF xyz location [meters]
         * outpus ::
         *  EL --> elevation angle [rad]
         */
        Vector3 posENU = ned2enu(p1);
        double El = std::atan2(posENU(2), posENU.norm());
        return El;
}

double calcEl(const Point3& p1, const Point3& p2){
        /*
           inputs ::
           p1 --> ECEF xyz satellite coordinates [meter]
           p2 ---> ECEF xyz receiver coordinates [meter]
           output ::
           El --> elevation angle [rad]
         */
        Vector3 posENU = xyz2enu(p1,p2);
        double El = std::atan2(posENU(2), posENU.norm());
        return El;
}

double tropMap(const double& El){
        /*
           inputs ::
           El --> receiver to satellite elevation angle [rad]
           output ::
           m --> troposphere delay map
         */
        double m = 1.001/sqrt(0.002001+( pow(sin(El), 2)));
        return m;
}

double tropDry(const Point3& p1){
        /*
           inputs ::
           p1 --> ECEF xyz receiver coordinated [meter]
           output ::
           tropDelta --> delay associated with troposphere [meter]
         */
        Vector3 posLLH = xyz2llh(p1);
        double recHeight = posLLH(2)/1000;         // receiver height [km]
        double pSea = stdPressure* pow( ( (stdTemp-(6.5*recHeight))/(stdTemp+6.5)), 5.2459587 );
        double tropDelta =  ( (1e-6/5) * ( (77.624*(pSea/(stdTemp+6.5*recHeight))) * (40136+148.72*((stdTemp-(6.5*recHeight))- 288.16)) ) );
        return tropDelta;
}

double dopplerObs(double r1, double r2){
        /*
           inputs ::
           r1 --> carrier phase obs. timestep n
           r2 --> carrier phase obs. timestep n-1
           output ::
           tdcp --> time differenced carrier phase observable
         */
        double tdcp = r1 - r2;
        return tdcp;
}

double elDepWeight(const Point3& p1, const Point3& p2, double measWeight) {
        /*
           inputs ::
           p1 --> ECEF xyz satellite coordinates [meter]
           p2 ---> ECEF xyz receiver coordinates [meter]
                              measNoise ---> initial noise applied to observable [meters]
           output ::
           r --> elevation angle dep. GNSS obs. weigth [rad]
         */
        double el = calcEl(p1,p2);
        double r = (1 / ( sin(el) )) * measWeight;
        return r;
}

Point3 satVelocity(const Point3& p1, const Point3& p2, double ts){
        /*
           inputs ::
           p1 --> sat xyz at timestep n
           p2 --> sat xyz at timestep n-1
           ts --> timestep
           outputs ::
           satVel --> sat velocity
         */

        Point3 satVel = ( (p1 - p2)/ (2*ts) );
        return satVel;
}

}
