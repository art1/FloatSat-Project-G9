
#pragma once

#include "math.h" // sin, sqrt, etc


#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

class Matrix3D;
class Matrix6D;
class Matrix4D;
class Quaternion;
class CoordinateFrame3D;
class Vector4D;
class Polar;
class AngleAxis;
class YPR;

/**
 * \defgroup A globale Funktionen und Konstanten*/

//======Konstanten======
///Konstante EPSILON wird benoetigt um Rundungsfehler bei der Gleitkommarithmetik nicht zu falschen Ergebnissen fuehren zu lassen
/** Soll beispielsweise gezeigt werden, dass 2 Vektoren orthogonal aufeinander stehen, so muss dessen Skalarprodukt 0 ergeben,
da Gleitkommaarithmetik nur eine begrenzte Genauigkeit erlaubt genuegt eine Approximation der 0 im EPSILON-Bereich, um zu zeigen,
dass 2 Vektoren den Winkel 90 Grad einschliessen.
\ingroup A */
const double EPSILON = 0.0000001;

/// iff abs value is less then EPSILON
inline bool isAlmost0(const double &a) { return fabs(a) < EPSILON; }
inline double grad2Rad(const double &grad) {return grad * M_PI / 180.0; }
inline double rad2Grad(const double &rad)  {return rad * 180.0 / M_PI; }
inline double square( const double &x ) { return x*x; }
inline double frac( const double &x ) { return x - floor(x); }
inline double mod( const double &dividend, const double &divisor ) { return (isAlmost0(divisor) ? 0.0 : divisor * frac( dividend/divisor )); }

//=====================================================Klassenbeschreibung Klasse Vector3D============================================
/// Die Klasse Vector3D stellt einen Vektor im 3-dimensionalen euklidischen Raum dar.
/** Die Klassen Vector3D speichert einen 3-dimensionalen Vektor durch 3 double-Werte, die jeweils die
x,y und z-Koordinate eines Vektors speichern. Die Klasse wird benoetigt um eine Rotation durch ein Quaternion
,ein YPR, ein AngleAxis oder eine Matrix3D durchzufuehren*/
class Vector3D {

//======Klassenvariablen======
public:
    double x;
    double y;
    double z;


//======Standardkonstruktor======
/// Der Standardkonstruktor, er erzeugt den Nullvektor (0,0,0)
    Vector3D();

//======3-double-Konstruktor======
/// Dieser Konstruktor erzeugt einen Vector3D aus 3 uebergebenden Parametern
    /** Es wird ein Vector3D erzeugt,indem 3 double Werte uebergeben werden, die in den jeweiligen Komponenten gespeichert
    werden, es wird also der Vektor (x,y,z) erzeugt*/
    Vector3D (const double &x, const double &y, const double &z);


//======Copy-Konstruktor======
/// Der Copy-Konstruktor erzeugt einen Vektor aus einem anderen Vektor
    /** Es wird ein anderer Vector3D uebergeben, und dessen x,y und z Koordinaten uebernommen und daraus
    der Vector3D (other.x,other.y,other.z) erzeugt*/
    Vector3D(const Vector3D& other);


//======Array-Konstruktor======
///Ein Konstkruktor fuer die Erzeugung eines Vektors aus einem double-Array
    /** Aus dem Array wird der Vector3D (arr[0],arr[1],arr[2]) erzeugt*/
    Vector3D (double* arr);


//======Vektoraddition======
/// Fuehrt die Vektoraddition aus
    /** es wird die komponentenweise Addition zweier Vektoren *this und other vorgenommen
    \return einen neuen Vector3D (this->x + other.x , this->y + other.y , this->z + other.z)*/
    Vector3D vecAdd(const Vector3D& other) const;


//======Vektorsubtraktion======
///Fuehrt die Vektorsubtraktion aus
    /** es wird die komponentenweise Subtraktion zweier Vektoren *this und other vorgenommen
    \return einen neuen Vector3D (this->x - other.x , this->y - other.y , this->z - other.z)*/
    Vector3D vecSub(const Vector3D& other) const;


//======S-Multiplikation======
/// Skaliert den Vektor mit einem Faktor
    /** Es wird ein neuer Vektor zurueckgegeben, der mit dem Skalar factor gestreckt wurde
    \return es wird also ein Vector3D(this->x*factor , this->y*factor , this->z*factor)erzeugt*/
    Vector3D  scale(const double &factor) const;


//======Kreuzprodukt======
///Erzeugt das Kreuzprodukt zwischen *this und other
    /** Es wird ein neuer Vector3D erzeugt, der das Kreuzprodukt (*this x right) enthaelt
    \return ein neuer Vector3D (*this x other)*/
    Vector3D cross(const Vector3D& right) const;


//======Skalarprodukt======
///Erzeugt das Skalarprodukt
    /** Es wird das Skalarprodukt *this * other errechnet
    \return ein double-Wert, mit dem Betrag *this * other*/
    double dot(const Vector3D& other) const;


//======Laenge eines Vektors======
///Berechnung der Laenge des Vectors3D
    /** Es wird der Betrag des *this-Vektors errechnet
    \return den double-Wert |*this|*/
    double getLen() const;


//======Normlisierung======
/// Vektor wird normalisiert
    /** Es wird der *this-Vektor mit Laenge 1 gebildet
    \return ein neuer Vector3D *this/(*this.getLength())*/
    Vector3D normalize() const;


//======isNormalized======
/// Es wird geprueft, ob der *this-Vektor die Laenge 1 besitzt
    /** Prueft, ob die Laenge des *this-Vektors im Bereich EPSILON um 1 liegt
    \return true, wenn abs(1-(*this).getLength())<= EPSILON, sonst false*/
    bool isNormalized() const;


//======equals======
/// Prueft, ob der Vektor *this mit dem Vektor other identisch ist.
    /** Zwei Vektoren sind genau dann identisch, wenn this->x == other.y, this->y == other.y
    und this->z == other.z \return true, wenn Identitaet vorliegt, false sonst*/
    bool equals(const Vector3D& other) const;


//======getAngle======
/// Berechnet den Winkel zwischen 2 Vektoren
    /** es wird der Winkel zwischen dem *this-Vektor und dem Vektor other errechnet
    \return Winkel zwischen den beiden Vektoren in Radians*/
    double getAngle(const Vector3D& other) const;


//======isOrthogonal======
/// Prueft, ob 2 Vektoren senkrecht zueinander stehen
    /** Es wird das Skalarprodukt zwischen den beiden Vektoren *this und other errechnet und geprueft, ob
    sich das Ergebnis in einer Epsilon-Umgebung um die 0 befindet.
    \return true, wenn abs(*this * other) <=Epsilon sonst false */
    bool isOrthogonal(const Vector3D& other) const;


//======carToPolar======
/// Die Koordinaten werden von kartesischen auf Kugelkoordinaten umgerechnet.
    /** Die Koordinaten des *this-Vektors (x,y,z)werden komponentenweise in Kugelkoordinaten(r,phi,theta)umgerechnet
    \return ein  Polar mit den Kugelkoordinaten(r,phi,theta), wobei phi und theta in Radians ausgegeben werden*/
    Polar carToPolar() const;


//======polarToCar======
///Die Koordinaten werden von Kugelkoordinaten auf kartesische Koordinaten umgerechnet.
    /** Aus den Kugelkoordinaten (r,phi,theta), wobei phi und theta in Radians uebergeben werden muessen werden kartesische Koordinaten (x,y,z)
    \return ein neuer Vector3D mit den kartesischen Koordinaten (x,y,z) aus den Kugelkoordinaten (r,phi,theta) */
    Vector3D polarToCar() const;


//======Vektor-Matrix-Multiplikation======
/// Die Multiplikation zwischen einer Matrix und einem Vektor
    /** Es wird die Multiplikation w = M * (*this) ausgefuehrt
    \return einen neuen Vector3D, der das Ergebnis der Multiplikation M *(*this) enthaelt*/
    Vector3D matVecMult(const Matrix3D& M) const;


//======Rotation mit einem Quaternion======
/// Eine Rotation des Vektors mit einem Quaternion
    /** Der Vektor *this wird durch ein Quaternion q im eigenen System rotiert. Es wird dabei
    der Quaternionrotationsoperator L_q(v)=qvq^* verwendet \return ein neuer Vector3D w = qvq^* */
    Vector3D qRotate(const Quaternion& q) const;


//======Rotation mit einer Rotationsmatrix======
///Eine Rotation des Vektors mit einer Matrix3D
    /** Der Vektor *this wird durch eine Rotationsmatrix M im eigenen System rotiert. Dies entspricht
    der Multiplikation des Vektors *this mit der Matrix \return ein neuer Vector3D w = M* v*/
    Vector3D mRotate(const Matrix3D& M) const;


//======Rotation mit der Rodriguez Formel======
/// Eine Rotation des Vektors durch eine Achse u und einen Winkel phi
    /** Die Rotation des Vektor *this erfolgt hierbei durch die Rodriguez Rotationsformel Ro_{u,phi}(v)
    \return ein neuer Vector3D w = Ro_{u,phi}(v)*/
    Vector3D aRotate(const AngleAxis& u_phi) const;

/// eine rotation aus ypr
    Vector3D yprRotate(const YPR& ypr) const;


//======to4D======
///Eine Umwandlung des 3D-Vektors in einen 4D-Vektor
    /** Es wird aus dem 3D-Vektor *this (x,y,z) ein neuer 4D-Vektor (x,y,z,1)erzeugt
    \return den Vector4D (x,y,z,1) */
    Vector4D to4D() const;


//======print======
///Eine Konsolenausgabe eine Vector3D
    /** Es wird eine Ausgabe des Vector3D (x,y,z) in der Form [x  y  z] erzeugt*/
    void print() const;
};

Vector3D elementWiseProduct(const Vector3D &left, const Vector3D &right);
Vector3D elementWiseDivision(const Vector3D &left, const Vector3D &right);

/*******************        operators       **********************/

inline Vector3D operator+   (const Vector3D &left, const Vector3D &right) { return left.vecAdd(right); }
inline Vector3D operator-   (const Vector3D &left, const Vector3D &right) { return left.vecSub(right); }
inline Vector3D operator*   (const double   &left, const Vector3D &right) { return right.scale(left); }
inline Vector3D operator*   (const Vector3D &left, const double   &right) { return left.scale(right); }
inline Vector3D operator/   (const Vector3D &left, const double   &right) { return left.scale(1.0/right); }
inline double   dotProduct  (const Vector3D &left, const Vector3D &right) { return left.dot(right);   }
inline Vector3D crossProduct(const Vector3D &left, const Vector3D &right) { return left.cross(right); }


//=====================================================Klassenbeschreibung Klasse Vector4D============================================
///Die Klasse Vector4D stellt die homogene Repraesentation eines 3D-Vektors dar
/** Eine homogene Darstellung eines 3D-Vektors (x,y,z) ist der 4D-Vektor (x,y,z,1), benoetigt wird diese Klasse um eine
 Moeglichkeit der Multiplikation eines Vektors mit einer 4D-Matrix zu erlauben, sie erbt die 3 Klassenvariablen x,y,z
 von der Klasse Vector3D */
class Vector4D : public Vector3D {

//======Klassenvariablen======
public:

/// die 4-te Komponente des homogenen Vektors, in der Regel = 1
    double scale;


//======Standardkonstruktor======
///Standardkonstruktor erzeugt den Vector4D (0,0,0,1)
    Vector4D();


//======4-double-Konstruktor======
///Erzeugt einen Vector4D (x,y,z,scale) aus 4 uebergebenden doubles (x,y,z,scale)
    Vector4D (const double &x, const double &y, const double &z, const double &scale);


//======Copy-Konstruktor======
///Copy-Konstruktor
    /** Es wird ein Vector4D aus einem anderen Vector4D erzeugt, der exakt die selben Komponenten aufweist*/
    Vector4D(const Vector4D& other);


//======Array-Konstruktor======
///Array-Konstruktor
    /** Es wird ein Vector4D(x,y,z,scale) aus dem Array arr wie folgt erzeugt : Vector4D(arr[0],arr[1],arr[2],arr[3])*/
    Vector4D (double* arr);


//======Matrix-Vektor-Multiplikation======
///Eine Multiplikation einer Matrix4D mit einem Vector4D
    /** \return ein Vector4D w = M*v */
    Vector4D matVecMult(const Matrix4D& M) const;


//======to3D======
///3D-Repraesentation eines Vector4D
    /** Es wird aus einem Vector4D (x,y,z,scale) eine 3D-Repraesentation erstellt \return einen Vector3D (x,y,z)*/
    Vector3D to3D() const;


//======Rotation und Translation======
///Eine Rotation und eine Translation des Vektors
    /** Eine Matrix4D stellt eine Rotation und einen Translation in einem dar, diese Methode fuehrt beides gemeinsam aus
    \return Einen Vector4D w, der sowohl durch die Matrix4D M rotiert, als auch verschoben wurde*/
    Vector4D mRotate(const Matrix4D& M) const;


//======print======
///Eine Konsolenausgabe eines Vector4D
    /** Es wird eine Ausgabe des Vector3D (x,y,z,scale) in der Form [x  y  z  scale] erzeugt*/
    void print() const;
};


//=====================================================Klassenbeschreibung Klasse Quaternion============================================
///Die Klasse Quaternion repraesentiert die klassischen Hamilton Quaternion
/** ein Hamilton Quaternion ist ein Quaternion der Form Q =q0+iq1+jq2+kq3= q0 +q , wobei q durch einen Vector3D und q0 durch
 ein double dargestellt werden*/
class Quaternion {

public:
//======Klassenvariablen======
/// q0 stellt den skalaren Anteil eines Quaternions dar.
    double q0;


/// q stellt den Vektoranteil eines Quaternions dar.
    Vector3D q;


//======Standarskonstruktor======
///Standardkonstruktor, erstellt das Einheitsquaternion q= (1,0,0,0)
    Quaternion();


//======Copy-Konstruktor======
///Copy-Konstruktor
    /** erzeugt ein Quaternion, identisch zu dem uebergebenden Quaternion q*/
    Quaternion(const Quaternion& q);


//======4-double-Konstruktor======
///Erzeugt ein Quaternion aus 4 uebergebenden double-Werten
    Quaternion(const double &q0, const double &q1, const double &q2, const double &q3);


//======Array-Konstruktor======
///Array-Konstruktor
    /**Erzeugt ein Quaternion q = (q0,q1,q2,q3) aus einem Array in der Art: (arr[0],arr[1],arr[2],arr[3])*/
    Quaternion(double* arr);


//======Skalar-Vektor-Konstruktor======
///Erzeugt ein Quaternion aus einem Skalarteil q0 und einem Vector3D q
    Quaternion(const double &q0,const Vector3D& q);


//======AngleAxis-Konstruktor======
///Erzeugt ein Quaternion aus einem AngleAxis
    /**Erzeugt in Quaternion q in folgender Art: q = cos(phi/2)+u*sin(phi/2)*/
    Quaternion(const AngleAxis& other);


//======Matrix-Konstruktor======
///Erzeugt ein Quaternion aus einer Matrix3D
    /**Das erzeugte Quaternion enthaelt die selben Informationen wie die uebergebende Matrix3D*/
    Quaternion(const Matrix3D& other);


//======YPR Konstruktor======
/// Erzeugt ein Quaternion aus Eulerwinkeln
    Quaternion(const YPR& other);


//======getAngle======
/// liefert den Rotationswinkel des Quaternion
    /**Es wird der Rotationswinkel aus dem Quaternion gewonnen \return  ein double, den Rotationswinkel phi in Radians*/
    double    getAngle() const;


//======getVec======
///liefert den Rotationsvektor des Quaternions
    /**Es wird die Rotationsachse des zugehörigen Quaternions errechnet \return einen Vector3D der die Rotationsachse repraesentiert*/
    Vector3D  getVec() const;


//======Quaternionaddition======
///Quaternionaddition
    /**Es wird eine komponentenweise Addition durchgefuehrt \return ein Quaternion *this + other*/
    Quaternion qAdd(const Quaternion& other) const;


//======Quaternionsubtraktion======
///Quaternionsubtraktion
    /**Es wird eine komponentenweise Subtraktion durchgefuehrt \return ein Quaternion *this - other*/
    Quaternion qSub(const Quaternion& other) const;


//======S-Multiplikation======
/// S-Multiplikation
    /** Es wird ein Quaternion erzeugt, welches auf Basis von *this eine Streckung um factor darstellt
    \return ein neues Quaternion skaliert mit faktor*/
    Quaternion scale(const double &factor) const;


//======Quaternion-Multiplikation======
///Quaternion-Multiplikation
    /**Die Quaternion-Multiplikation zweiter Quaternion *this * other \return ein neues Quaternion r = this * other*/
    Quaternion qMult(const Quaternion& right) const;


//======Quaternion-konjugation======
///Quaternion-Konjugation
    /**Das komplex konjugierte eines Quaternion q = q0 + q stellt ein Quaternion q^*= q0 -q dar.\return ein neues Quaternion q^* = q0-q*/
    Quaternion conjugate() const;


//======Quaternion Inversion======
///Quaternion Inversion
    /**Invertiert das *this Quaternion \return Ein neues Quaternion, dass das multiplikative Inverse von *this darstellt.*/
    Quaternion invert() const;


//======Quaternion-Division======
///Quaternion-Division
    /**Es wird die Division auf Basis des multiplikativ Inversen durchgefuehrt \return Ein neues Quaternion (*this)/(other.invert())*/
    Quaternion qDivide(const Quaternion& denominator) const;


//======Quaternion-Betrag======
///Errechnet den Betrag des Quaternions
    /**Auf Basis der euklidischen Norm wird der Betrag des *this Quaternions erzeugt. \return Ein double mit dem Betrag des *this Quaternions*/
    double getLen() const;


//======Einheitsquaternion======
///Erzeugt ein Einheitsquaternion
    /**Es wird ein Quaternion der Lanege 1 erzeugt, welches noetig zum Rotieren ist. \return ein neues Einheitsquaternion*/
    Quaternion normalize() const;


//======isNormalized======
///Prueft auf Einheitsquaternion
    /**Es wird innerhalb einer EPSILON-Umgebung geprueft, ob das Quaternion ein Einheitsquaternion darstellt.\return true, wenn die Laenge
    des *this Quaternion innerhalb der Epsilon Umgebung liegt, false sonst*/
    bool isNormalized() const;


//======equals======
///Prueft auf Identitaet
    /**Zwei Quaternionen sind genau dann identisch, wenn alle ihre Komponenten ueberein stimmen \return true, wenn beide Quaternione
    uebereinstimmen, false sonst.*/
    bool equals(const Quaternion& other) const;


//======toAngleAxis======
///Erzeugt die AngleAxis repraesentation des Quaternion
    /**Aus dem Quaternion wird die Rotationsachse und der Rotationswinkel gewonnen und ein AngleAxis erzeugt.\return die AngleAxis
    Repraesentation des Quaternions *this*/
    AngleAxis toAngleAxis() const;


//======toMatrix3D======
///Erzeugt die Rotationsmatrix aus dem Quaternion
    /**Das Quaternion wird in die zugehoerige Rotationsmatrix umgewandelt. \return die Matrix3D Repraesentation des Quaternion *this*/
    Matrix3D toMatrix3D() const;


//======toYPR======
///Erzeugt ein YPR aus dem Quaternion
    /**Aus dem Quaternion werden die Eulerwinkel (Yaw,Pitch,Roll) gewonnen. \return die YPR Repraesentation des Quaternion *this*/
    YPR toYPR() const;
    YPR toYPRnils() const;

//======print======
///Eine Konsolenausgabe eines Quaternions
    /** Es wird eine Ausgabe des Quaternion (q0,q1,q2,q3) in der Form [q0  q1  q2  q3] erzeugt*/
    void print() const;
};

/*******************        operators       **********************/

inline Quaternion operator+(const Quaternion &left, const Quaternion &right)  { return left.qAdd(right); }
inline Quaternion operator-(const Quaternion &left, const Quaternion &right)  { return left.qSub(right); }
inline Quaternion operator*(const Quaternion &left, const Quaternion &right)  { return left.qMult(right); }
inline Quaternion operator*(const double     &left, const Quaternion &right)  { return right.scale(left); }
inline Quaternion operator*(const Quaternion &left, const double     &right)  { return left.scale(right); }
inline Vector3D   operator*(const Quaternion &left, const Vector3D   &right)  { return right.qRotate(left); }
inline Quaternion operator/(const Quaternion &left, const double     &right)  { return left.scale(1.0/right); }
inline Quaternion operator-(const Quaternion &right)                   { return right.conjugate(); }

inline Quaternion qX(const double &phi) { return Quaternion(cos(phi/2), sin(phi/2), 0.0, 0.0); }
inline Quaternion qY(const double &phi) { return Quaternion(cos(phi/2), 0.0, sin(phi/2), 0.0); }
inline Quaternion qZ(const double &phi) { return Quaternion(cos(phi/2), 0.0, 0.0, sin(phi/2)); }
inline Quaternion q1()           { return Quaternion(1,          0.0, 0.0, 0.0); }
Quaternion operator*(const Matrix4D &left, const Quaternion &right);



//=====================================================Klassenbeschreibung Klasse Matrix3D============================================
///Die Klasse Matrix3D stellt eine 3x3 Matrix im euklidischen Raum dar.
/**Zur Speicherung der Matrixelemente wird ein 2-dimensionales double-Array verwendet */
class Matrix3D {

public:

///2-dimensionales Array zur Speicherung der Matrixelemente
    double r[3][3];

//====Konstruktoren====
///Standardkonstruktor
    /**Erstellt die Einheitsmatrix*/
    Matrix3D();


///Spaltenkonstruktor
    /**Aus 3 uebergebenden Vektoren wird eine Matrix aufgebaut, indem die Vektoren der Reihe nach als Spalten eingesetzt werden*/
    Matrix3D(const Vector3D& column1, const Vector3D& column2, const Vector3D& column3);


///Array-Konstruktor
    /**Aus einem Array wird die Matrix von links oben nach rechts unten Schrittweise gefuellt*/
    Matrix3D(double* arr);


///Copy-Konstruktor
    /**Dieser Konstruktor erzeugt eine Kopie der Matrix other*/
    Matrix3D(const Matrix3D& other);


///Vektor-Konstruktor
    /**Ein Vector3D wird dazu benutzt, um eine DiagonalMatrix mit den Eintraegen des Vektors zu erzeugen*/
    Matrix3D(const Vector3D& init);  //diagonalmatrix aus vektor


/// YPR-Konstruktor
    /**Eine Matrix wird aus den Eulerwinkeln Yaw,Pitch und Roll erzeugt*/
    Matrix3D(const YPR& other);


///AngleAxis Konstruktor
    /**Erzeugt eine Matrix aus eine AngleAxis*/
    Matrix3D(const AngleAxis& other);


///Quaternion Konstruktor
    /**Erzeugt eine Matrix3D aus einem Quaternion*/
    Matrix3D(const Quaternion& other);


///Liefert den Rotationsvektor der Matrix
    /**Jede Rotationsmatrix stellt eine Rotation um eine Achse mit einem Winkel dar.
    \return Ein Vector3D , der die Rotationsachse der Matrix3D enthaelt*/
    Vector3D getVec() const;

///Liefert den Rotationswinkel der Matrix
    /**Jede Rotationsmatrix  stellt eine Rotation um eine Achse mit einem Winkel dar.
    \return Ein double , der den Rotationswinkel in Radians der Matrix3D enthaelt*/
    double getAngle() const;


//====Getter für Spalten bzw Zeilen ====
///Liefert die erste Zeile der Matrix3D
    /**\return einen Vector3D mit der ersten Zeile der Matrix*/
    Vector3D getRow1() const;


///Liefert die zweite Zeile der Matrix3D
    /**\return einen Vector3D mit der zweiten Zeile der Matrix*/
    Vector3D getRow2() const;


///Liefert die dritte Zeile der Matrix3D
    /**\return einen Vector3D mit der dritten Zeile der Matrix*/
    Vector3D getRow3() const;


///Liefert die erste Spalte der Matrix3D
    /**\return einen Vector3D mit der ersten Spalte der Matrix*/
    Vector3D getColumn1() const;


///Liefert die zweite Spalte der Matrix3D
    /**\return einen Vector3D mit der zweiten Spalte der Matrix*/
    Vector3D getColumn2() const;


///Liefert die dritte Spalte der Matrix3D
    /**\return einen Vector3D mit der dritten Spalte der Matrix*/
    Vector3D getColumn3() const;


//====Setter für Spalten und Reihen =====
/// Aendert die erste Zeile der Matrix3D auf den Vector3D row
    void setRow1(const Vector3D& row);


/// Aendert die zweite Zeile der Matrix3D auf den Vector3D row
    void setRow2(const Vector3D& row);


/// Aendert die dritte Zeile der Matrix3D auf den Vector3D row
    void setRow3(const Vector3D& row);


/// Aendert die erste Spalte der Matrix3D auf den Vector3D column
    void setColumn1(const Vector3D& column);


/// Aendert die zweite Spalte der Matrix3D auf den Vector3D column
    void setColumn2(const Vector3D& column);


/// Aendert die dritte Spalte der Matrix3D auf den Vector3D column
    void setColumn3(const Vector3D& column);


//====mAdd,mSub,scale,mMult ====
///Matrixaddition
    /**Fuehrt eine Komponentenweise Matrixaddition durch
    \return eine neue Matrix3D mit der Summe von *this und other*/
    Matrix3D mAdd(const Matrix3D& other) const;


///Matrixsubtraktion
    /**Fuehrt eine Komponentenweise Matrixsubtraktion durch
    \return eine neue Matrix3D mit der Differenz von *this und other*/
    Matrix3D mSub(const Matrix3D& other) const;


///Matrixskalierung
    /**Fuehrt eine Komponentenweise Skalierung aller Eintraege der Matrix mit factor durch.
    \return eine neue Matrix3D skaliert mit factor*/
    Matrix3D scale(const double &factor) const;


///Matrixmultiplikation
    /**Fuehrt die Matrixmultiplikation im R^3 durch.
     \return eine neue Matrix3D *this * other*/
    Matrix3D mMult(const Matrix3D& other) const;
//==== invert,transpose,mDivide====


///Kofaktormatrix
    /**Erstellt eine Matrix3D, die anstatt jedes Eintrages aij die Unterdeterminante Aij enthaelt, wird benoetigt um die inverse
    zu berechnen. \return Eine neue Matrix3D , mit den Unterdeterminanten als Eintraege*/
    Matrix3D cofac() const;


///Adjunkte
    /**Erstellt die Adjunkte-Matrix, diese stellt einfach die Transponierte Kofaktormatrix dar.
    \return. eine neue Matrix3D , die Adjunkte zu *this */
    Matrix3D adjoint() const;


///Matrix-Invertierung
    /**Errechnet das Inverse der *this Matrix. \return Die inverse Matrix3D zu *this*/
    Matrix3D invert() const;


///Transponierte Matrix
    /**Transponiert die *this Matrix. \return Die transponierte Matrix3D von *this*/
    Matrix3D transpose() const;

///Spur der Matrix
    /**Liefert die Spur der *this Matrix. \return Die Spur der Matrix *this*/
    double trace() const;


///Matrix-Division
    /**Die Matrixdivision ist eine Multiplikation mit dem Inversen. Errechnet *this * other.invert() . \return
    eine Matrix3D , das Ergebnis der Division.*/
    Matrix3D mDivide(const Matrix3D& other) const;


//====Fundamentalrotationen====
/// Erstellt die Fundamenalmatrix zur Rotation um die x-Achse
    /**\return Eine Matrix3D ,die Fundamentalmatrix um die x-Achse mit dem Winkel angle(in Radians)*/
    void rotationX(const double &angle);


/// Erstellt die Fundamenalmatrix zur Rotation um die y-Achse
    /**\return Eine Matrix3D ,die Fundamentalmatrix um die y-Achse mit dem Winkel angle(in Radians)*/
    void rotationY(const double &angle);


/// Erstellt die Fundamenalmatrix zur Rotation um die z-Achse
    /**\return Eine Matrix3D ,die Fundamentalmatrix um die z-Achse mit dem Winkel angle(in Radians)*/
    void rotationZ(const double &angle);


//==== determinant,isOrthogonal ,equals====
///Determinante
    /**Errechnet die Determinante der Matrix3D. \return ein double mit der Determinante der *this-Matrix*/
    double determinant() const;


///Prueft auf orthogonalitaet
    /**Eine Matrix ist orthogonal, wenn sie mit ihrem Transponierten multipliziert die Einheitsmatrix ergibt.
    \return true , wenn orthogonal , false sonst*/
    bool isOrthogonal() const;


///Prueft auf Identitaet
    /**\return true , falls beide Matrizen identisch sind , false sonst*/
    bool equals(const Matrix3D& other) const;


//==== toQuaternion====
/// Umwandlung in korrespondierendes Quaternion
    /**Die vorliegende Matrix3D wird in das zugehoerige Quaternion gewandelt \return ein Quaternion ,als Repraesentant der Matrix3D*/
    Quaternion toQuaternion() const;


/// Umwandlung in korrespondierendes YPR
    /**Die vorliegende Matrix3D wird in das zugehoerige YPR gewandelt \return ein YPR ,als Repraesentant der Matrix3D*/
    YPR toYPR() const;


/// Umwandlung in korrespondierendes AngleAxis
    /**Die vorliegende Matrix3D wird in das zugehoerige AngleAxis gewandelt \return ein AngleAxis ,als Repraesentant der Matrix3D*/
    AngleAxis toAngleAxis() const;


///Eine Konsolenausgabe einer Matrix3D
    /** Erzeugt eine Ausgabe der Matrix3D  */
    void print() const;
};


/*******************        operators       **********************/

inline Matrix3D operator+(const Matrix3D &left, const Matrix3D &right) { return left.mAdd(right); }
inline Matrix3D operator-(const Matrix3D &left, const Matrix3D &right) { return left.mSub(right); }
inline Matrix3D operator*(const Matrix3D &left, const Matrix3D &right) { return left.mMult(right); } 

inline Matrix3D operator*(const double   &left, const Matrix3D &right) { return right.scale(left); }
inline Matrix3D operator*(const Matrix3D &left, const double   &right) { return left.scale(right); }
inline Matrix3D operator/(const Matrix3D &left, const double   &right) { return left.scale(1.0/right); }

inline Matrix3D operator*   (const Vector3D &left, const Vector3D &right) { return Matrix3D( left*right.x, left*right.y, left*right.z ); }

inline Vector3D operator*(const Matrix3D &left, const Vector3D &right) { return right.matVecMult(left); } // in matVecMutl right and left are toggled

//=====================================================Klassenbeschreibung Klasse YPR============================================
/// Diese Klasse stellt ein Tripel bestehend aus den Eulerwinkeln Yaw,Pitch,Roll dar.
/**Es wird basierend auf der Eulersequenz ZYX ein Tripel YPR erzeugt,gespeichert durch 3 doubles
yaw,pitch und roll, alle Angaben der Winkel muessen im Bogenmass erfolgen*/

#define RPY YPR
class YPR {
public:
/// Double zum Speichern des Yaw-Winkels alpha in Radians.
    double yaw;


/// Double zum Speichern des Pitch-Winkels beta in Radians.
    double pitch;


///  Double zum Speichern des Roll-Winkels gamma in Radians.
    double roll;


//==== Konstruktoren=====
///Standardkonstruktor
    /**Es wird ein YPR (0,0,0) erzeugt*/
    YPR();


///Copy-Konstruktor
    /**Es wird ein YPR erzeugt, welches identisch zu other ist.*/
    YPR(const YPR& other);


/// Double-Konstruktor
    /**Es wird ein YPR aus den 3 uebergenden doubles yaw(in Rad), pitch(in Rad) und roll(in Rad) erzeugt,*/
    YPR(const double &yaw, const double &pitch, const double &roll);

/// Quaternion-Konstruktor
    /**es wird ein YPR aus dem uebergebenem Quaternion q erzeugt*/
    YPR(const Quaternion& q);

/// Matrix3D-Konstruktor
    /**Es wird ein YPR aus einer Matrix3D erzeugt*/
    YPR(const Matrix3D& M);


///AngleAxis-Konstruktor
    /**Es wird ein YPR aus einem AngleAxis erzeugt*/
    YPR(const AngleAxis& other);


/// Umwandlung in korrespondierende Matrix3D
    /**Das vorliegende YPR wird in die dazugehoerige Rotationsmatrix gewandelt \return eine Matrix3D ,als Repraesentant des YPR*/
    Matrix3D toMatrix3D() const;


/// Umwandlung in korrespondierendes Quaternion
    /**Das vorliegende YPR wird in das dazugehoerige Quaternion gewandelt \return ein Einheits-Quaternion ,als Repraesentant des YPR*/
    Quaternion toQuaternion() const;


/// Umwandlung in korrespondierendes AngleAxis
    /**Das vorliegende YPR wird in das dazugehoerige AngleAxis gewandelt \return ein AngleAxis ,als Repraesentant des YPR*/
    AngleAxis toAngleAxis() const;


//======print======
///Eine Konsolenausgabe eines YPR
    /** Es wird eine Ausgabe des YPR in der Form [Yaw Pitch Roll] in Grad erzeugt*/
    void print() const;
};
//=====================================================Klassenbeschreibung Klasse AngleAxis============================================
///Diese Klasse stellt eine Rotation durch einen Drehwinkel und eine Rotationsachse dar
/**Eine der 4 Moeglichkeiten eine Rotation zu Repraesentieren ist eine Drehachse anzugeben, die hier als Vector3D
gespeichert wird und einen Drehwinkel(in Radians) anzugeben, ein double*/
class AngleAxis {
public:
/// Stellt den Einheitsvektor u der Rotation dar.
    Vector3D u;

/// Stellt den Rotationswinkel phi in Radians dar.
    double phi;


//==== Konstruktoren=====
///Standardkonstruktor
    /**Erstellt das AngleAxis([0,0,0],0)*/
    AngleAxis();

/// from phi, x,y,z
    AngleAxis(const double &phi, const double &x, const double &y, const double &z);

///Copy-Konstruktor
    /**Es wird ein AngleAxis erzeugt, welches identisch zu other ist.*/
    AngleAxis(const AngleAxis& other);

///Achsen-Winkel-Konstruktor
    /**Es wird ein AngleAxis erzeugt, welches eine Rotation um die Achse u mit dem Winkel phi(in Rad) darstellt.*/
    AngleAxis(const double &phi, const Vector3D& u);


///Quaternion-Konstruktor
    /**es wird ein AngleAxis aus dem uebergebenem Quaternion erzeugt*/
    AngleAxis(const Quaternion& q);


/// Matrix3D-Konstruktor
    /**es wird ein AngleAxis aus der uebergebenen Matrix3D erzeugt*/
    AngleAxis(const Matrix3D& M);


/// YPR-Konstruktor
    /**es wird ein AngleAxis aus dem uebergebenem YPR erzeugt*/
    AngleAxis(const YPR& ypr);


/// Umwandlung in korrespondierendes Quaternion
    /**Das vorliegende AngleAxis wird in das dazugehoerige Quaternion gewandelt \return ein Quaternion ,als Repraesentant des AngleAxis*/
    Quaternion toQuaternion() const;


/// Umwandlung in korrespondierende Matrix3D
    /**Das vorliegende YPR wird in die dazugehoerige Rotationsmatrix gewandelt \return eine Matrix3D ,als Repraesentant des AngleAxis*/
    Matrix3D toMatrix3D() const;


/// Umwandlung in korrespondierendes YPR
    /**Das vorliegende AngleAxis wird in das dazugehoerige YPR gewandelt \return ein YPR ,als Repraesentant des AngleAxis*/
    YPR toYPR() const;




///Eine Konsolenausgabe eines AngleAxis
    /** Es wird eine Ausgabe des AngleAxis (phi,u) in der Form [phi  u.x  u.y  u.z] erzeugt. Phi wird dabei Grad ausgegeben*/
    void print() const;

};


//=====================================================Klassenbeschreibung Klasse Matrix4D============================================
/// Die Klasse Matrix4D ist eine Implementierung der homogenen Koordinaten
/**Eine Matrix4D ist einen 4x4-Matrix die in erster Linie dazu dient, eine gleichzeitige Abhandlung einer Rotation
 und einer Translation zu ermoeglichen*/
class Matrix4D {


public:


/// Eine 2-dimensionales double-Array speichert die 16 Elemente in der 4x4 Matrix4D
    double r[4][4];


//====Konstruktoren====
///Standardkonstruktor
    /**Erstellt eine Matrix4D, die eine Tranlation von (0,0,0) und eine Rotation mit der Identitaetsmatrix ausfuehrt*/
    Matrix4D();


///Konstruktor aus Rotation und Translation
    /**Dieser Konstruktor erstellt eine Matrix4D, die eine Rotation rot und eine translation trans repraesentiert*/
    Matrix4D(const Matrix3D& rot, const Vector3D& trans);


///Array-konstruktor
    /**Erstellt eine Matrix4D aus einem double-Array, wobei zur Konstruktion die ersten 16 eintraege des Array wie folgt verwendet werden:
    Zunaechst wird der Eintrag oben links besetzt, dann von links nach rechts die erste Zeile der Matrix4D,anschließend werden in gleicher Weise
    als naechstes die 2te Zeile, die 3te und die 4te besetzt*/
    Matrix4D(double* arr);


///Copy-Konstruktor
    /**Erzeugt eine Matrix4D als identische Kopie einer anderen Matrix4D*/
    Matrix4D(const Matrix4D& other);


//====scale,mMult ====
///Liefert die Rotationsmatrix aus der Matrix4D
    /**Da jede Matrix4D eine Rotation und eine Translation zugleich darstellt, wird hier die Rotationsmatrix ausgegeben.
    \return Eine Matrix3D , welche die Rotation der Matrix4D repraesentiert.*/
    Matrix3D getRotation() const;

///Liefert die Translation aus der Matrix4D
    /**Da jede Matrix4D eine Rotation und eine Translation zugleich darstellt, wird hier der Translationsvektor ausgegeben.
    \return Ein Vector3D , welcher die Translation der Matrix4D repraesentiert.*/
    Vector3D getTranslation() const;


///Skaliert die Hauptdiagonale der Matrix4D mit factor
    /**Um einen Vektor zusaetzlich zur Rotation und Translation mit einer homogenen Matrix4D noch zu Skalieren wird die Hauptdiagonale
    der Rotationsmatrix skaliert.
     \return Eine Matrix4D, bei der die Hauptdiagonale der Rotationsmatrix mit factor multipliziert ist*/
    Matrix4D scale(const double &factor) const;


///Matrixmultiplikation
    /**Die Matrixmultiplikation einer 4x4 Matrix *this mit einer anderen 4x4 Matrix4D other, stellt eine
    regulaere Matrixmultiplikation dar. \return eine Matrix4D M der Art:M = *this * other */
    Matrix4D mMult(const Matrix4D& right) const;


//==== invert,transpose====
///Invertierung der Matrix
    /**Die Matrix4D wird der Art invertiert, so dass eine zuvor ausgefuehrte Rotation und Translation rueckgaengig gemacht werden kann.
    \return Eine neue Matrix4D, die inverse Matrix4D zu  *this */
    Matrix4D invert() const;


///Eine Konsolenausgabe einer Matrix4D
    /** Es wird eine Ausgabe der Matrix4D erzeugt*/
    void print() const;

};
//=====================================================Klassenbeschreibung Klasse CoordinateFrame3D============================================
///Diese Klasse ist eine Darstellung eines karstesischen Koordinatensystems im euklidischem Raum
/**Fuer ein kartesisches Koordinatensystem werden 3 orthogonale Einheitsvektoren x,y und z benoetigt, die hier
durch Objekte der Klasse Vector3D dargestellt sind, zusaetzlich ist ein Urpsrung in Form eines vierten Vector3D noetig um
eine eindeutige Repraesentation eines Koordinatensystems zu ermoeglichen*/
class CoordinateFrame3D {
public:

/// Einheitsvektor in x-Richtung
    Vector3D x;


///Einehitsvektor in y-Richtung
    Vector3D y;


///Einheitsvektor in z-Richtung
    Vector3D z;


/// Ursprung des Koordinatensystems
    Vector3D origin;


//====Konstruktoren====
///Standardkonstruktor
    /**Erstellt ein Koordinatensystem mit 4 [0 0 0]-Vektoren*/
    CoordinateFrame3D();


///Konstruktor aus 4 Vektoren
    /**Ein Koordinatensystem wird durch 3 Einheitsvektoren x,y,z und einem Ursprung origin erstellt*/
    CoordinateFrame3D(const Vector3D& x, const Vector3D& y, const Vector3D& z, const Vector3D& origin);


///Konstruktor aus 3 Vektoren
    /**Dieser Konstruktor wird beim TRIAD-Algorithmus verwendet, aus 2 beliebigen Vektoren wird ein orthogonales Set
    3-er Vektoren erstellt, der Ursprung wird auf origin gesetzt.*/
    CoordinateFrame3D(const Vector3D& x, const Vector3D& y, const Vector3D& origin);


///Copy-Konstruktor
    /**Erstellt ein CoordinateFrame3D als Kopie eines anderen CoordinateFrame3D other*/
    CoordinateFrame3D(const CoordinateFrame3D& other);


//====methoden=======
///Liefert die homogene-Transformationsmatrix zur Transformation zwischen 2 Koordinatensystemen
    /**Mit Hilfe der Richtungskosinus-Matrix kann eine Rotationsmatrix aus 2 gegebenen Koordinatensystemen erstellt werden,
    die Differenz der Urspruenge der Systeme wird als Translationsvektor genutzt. \return eine Matrix4D, die Vektoren zwischen 2 Koordinatensystemen
    umrechnen laesst*/
    Matrix4D mapTo(const CoordinateFrame3D& other) const;


//====rotate,translate====
/// Verschiebt das Koordinatensystem um den Vector3D trans
    /**Eine Translation eines frames kann als Translation seines Ursprunges verstanden werden.
     \return den um den Vector3D trans verschobenen CoordinateFrame3D*/
    CoordinateFrame3D translate(const Vector3D& trans) const;


///Rotiert den CoordinateFrame3D mit der Matrix3D rot
    /**Eine Rotation eines Koordinatensystems ist eine Rotation aller seiner Achsen, der Ursprung bleibt unveraendert
    \return ein neuer, mit der Matrix3D rotierter CoordinateFrame3D*/
    CoordinateFrame3D rotate(const Matrix3D& rot) const;


///Rotiert den CoordinateFrame3D mit dem Quaternion q
    /**Eine Rotation eines Koordinatensystems ist eine Rotation aller seiner Achsen, der Ursprung bleibt unveraendert
    \return ein neuer, mit dem Quaternion q rotierter CoordinateFrame3D*/
    CoordinateFrame3D rotate(const Quaternion& q) const;
//====mapTo=======



};

//=====================================================Klassenbeschreibung Klasse Complex============================================
///Die Klasse Complex stellt eine Implementierung der komplexen Zahlen dar.
/**Eine komplexe Zahl besteht aus z = a +ib , wobei a und b 2 reelle Zahlen sind und als double dargestellt werden*/
class Complex {
public:

///Realteil der komplexen Zahl
    double Re;

///Imaginaerteil der komplexen Zahl
    double Im;

///Standardkonstruktor
    /**Erstellt die komplexe Zahl z = 0 +i*0*/
    Complex();

///Copy-Konstruktor
    /**Erstellt eine komplexe Zahl als identische Kopie von other*/
    Complex(const Complex& other);

///Real-und Imaginaerteil-Konstruktor
    /**Erstellt eine komplexe Zahl mit Realteil Re und Imaginaerteil Im*/
    Complex(const double &Re, const double &Im);

///komplexe Addition
    /**Eine komplexe Addition stellt eine Komponentenweise Addition des Realteils bzw des Imaginaerteils dar.
    \return ein neues Complex mit *this + other*/
    Complex cAdd(const Complex& other) const;

///komplexe Subtraktion
    /**Eine komplexe Subtraktion stellt eine Komponentenweise Subtraktion des Realteils bzw des Imaginaerteils dar.
    \return ein neues Complex mit *this - other*/
    Complex cSub(const Complex& other) const;

///Skalarmultiplikation
    /**Eine Multiplikation eines Skalares mit einer komplexen Zalhl entspricht eine Multiplikation von scale mit dem Realteil
    und eine Multiplikation von scale mit dem Imaginaerteil. \return ein neues Complex gestreckt um scale.*/
    Complex cScale(const double &scale) const;

///komplexe Multiplikation
    /**Es wird die Multipliaktion auf dem Koerper C ausgefuehrt. \return ein neues Complex z = *this * other*/
    Complex cMult(const Complex& other) const;

///komplexe Potenzierung
    /**Liefert alle  ganzzahligen Potenzen von komplexen Zahlen. \return ein neues Complex z = (a+bi)^exponent*/
    Complex cPow(const int &exponent) const;

///komplexe E-Funktion
    /**Eine Implementierung der komplexen E-Funktion mit Hilfe der eulerschen Formel.
    \return ein neues Complex z = e^(*this) */
    Complex cExp() const;
};

inline Complex operator+ (const Complex &left, const Complex &right) { return left.cAdd(right); }
inline Complex operator- (const Complex &left, const Complex &right) { return left.cSub(right); }
inline Complex operator* (const double  &left, const Complex &right) { return right.cScale(left); }
inline Complex operator* (const Complex &left, const double  &right) { return left.cScale(right); }
inline Complex operator/ (const Complex &left, const double  &right) { return left.cScale(1.0/right); }



//=====================================================Klassenbeschreibung Klasse Polar============================================


/// Die Klasse Polar beschreibt die Darstellung eines Vektors im 3-dimensionalen euklidischen Raum in Polarkoordinaten(Kugelkoordinaten) durch (r,phi,theta).
/**Intern werden Polarkoordinaten mit 3 double-Werten gespeichert, der erste Parameter ist r , die andren beiden phi und theta. Die Winkel
sind dabei stets im Bogenmass zu uebergeben*/


class Polar {

public:
/// Variable zum Speichern der r-koordinate
    double r;

///Variable zum Speichern des Winkels phi in Radians
    double phi;

/// Variable zum Speichern des Winkels theta in Radians
    double theta;

///Standardkonstruktor
    /**Erstellt ein Polar (0,0,0)*/
    Polar();

///Double-Konstruktor
    /**Erzeugt ein Polar durch 3 doubles (r,phi,theta)*/
    Polar(const double &r, const double &phi, const double &theta);

///Copy-Konstruktor
    /**Erzeugt ein Polar als identische Kopie eines anderen Polar other*/
    Polar(const Polar& other);

///Vector3D Konstruktor
    /**Erzeugt ein Polar aus einem kartesischen Vector3D(x,y,z)*/
    Polar(const Vector3D& other);


///Wandelt Polar in kartesische koordinaten
    /**Es werden die Kugelkoordinaten (r,phi,theta)in 3-dimensionale kartesische Koordinaten verwandelt.
    \return ein Vector3D mit den kartesischen Koordinaten des *this Polar*/
    Vector3D toCartesian() const;

/// Erzeugt eine Konsolenausgabe eines Polar
    /**Es wird eine Ausgabe der Form [r=  phi=  theta=] erzeugt, obwohl intern alles in Radians gespeichert ist, erfolgt
    die Ausgabe der Winkel hier in Grad.*/
    void print() const;
};

//===================Deklaration Globale Funktionen============================================

//======FMod2p======
///Die Funktion liefert den Gleitkomma Rest, der bei der Division mit 2Pi bleibt
/**Es wird der Gleitkomma-Rest der bei der Division des doubles x mit 2PI bleibt im Intervall zwischen 0 und 2Pi errechnet.
\return Gleitkomma-Rest bei Division durch 2PI im Intervall [0,2Pi]
\ingroup A  */
double FMod2p( const double &x);

//======RotateX======
///Rotiert einen Vektor s um die x-Achse mit dem Winkel angle(in Rad)
/**\return einen um die x-Achse, mit dem Winkel angle rotierten Vector3D
\ingroup A  */
Vector3D rotateX( const Vector3D& s, const double &angle);

//double fabs(double value);
//======RotateY======
///Rotiert einen Vektor s um die y-Achse mit dem Winkel angle(in Rad)
/**\return einen um die y-Achse, mit dem Winkel angle rotierten Vector3D
\ingroup A  */
Vector3D rotateY( const Vector3D& s, const double &angle);


//======RotateZ======
///Rotiert einen Vektor s um die z-Achse mit dem Winkel angle(in Rad)
/**\return einen um die z-Achse, mit dem Winkel angle rotierten Vector3D
\ingroup A  */
Vector3D rotateZ( const Vector3D& s, const double &angle);


//======Erdkruemmungsradius======
///Errechnet den Erdkruemmungsradius in Metern.
/**Aus dem Breitengrad (Angabe in Grad)wird der nach WGS84 festgelegte Erdkruemmungsradius in Metern errechnet.
\return ein double mit dem Erdkruemmungsradius in Metern.
\ingroup A  */
double R_n(const double &angle);


//======Geodetic to ECEF======
/// Transformiert den Vector3D other von geodetischen Koordinaten in ECEF Koordinaten
/**Eingabe erfolgt in (phi,lambda,h) (Grad,Grad,Meter).
 \return Vector3D in kartesischen ECEF Koordinaten in (Meter,Meter,Meter)
 \ingroup A  */
Vector3D geodeticToECEF(const Vector3D& other);


//======ECEF to Geodetic======
/// Transformiert den Vector3D other von ECEF Koordinaten in geodetische Koordinaten
/**Eingabe erfolgt in (Meter,Meter,Meter).
 \return Vector3D in geodetischen Koordinaten in (phi,lambda,h) (Grad,Grad,Meter)
\ingroup A  */
Vector3D ecfToGeodetic(const Vector3D& other);



//=====================================================Vector3D============================================
Matrix3D skewSymmetricMatrix( const Vector3D &v );


//=====================================================Matrix4D============================================

Matrix4D operator+(const Matrix4D &left, const Matrix4D &right);
Matrix4D operator*(const Matrix4D &left, const double   &right);
Matrix4D operator/(const Matrix4D &left, const double   &right);

//=====================================================Vector6D============================================

class Vector6D {

  public:
    double v[6];

    Vector6D();
    Vector6D( const Vector6D& other );
    Vector6D( const double* arr );
    Vector6D( double x_, double y_, double z_,
              double u_, double v_, double w_ );
    Vector6D( const Vector3D &upper, const Vector3D &lower );

    double getLen() const;
    Vector6D scale(const double &factor) const;
    Vector6D vecAdd(const Vector6D& other) const;
    Vector6D vecSub(const Vector6D& other) const;
    Vector6D matVecMult(const Matrix6D& M) const;
};

/*******************        operators       **********************/
inline Vector6D operator+(const Vector6D& left, const Vector6D& right) { return left.vecAdd(right); }
inline Vector6D operator-(const Vector6D& left, const Vector6D& right) { return left.vecSub(right); }
inline Vector6D operator*(double value, const Vector6D& right) { return right.scale(value); }
inline Vector6D operator*(const Vector6D& left, const double &value) { return left.scale(value); }
inline Vector6D operator/(const Vector6D &left, const double &right) { return left.scale(1.0/right); }

/*******************        functions       **********************/
double dotProduct(const Vector6D& left, const Vector6D& right);

//=====================================================Matrix6D============================================

class Matrix6D {

public:
    double r[6][6];

    Matrix6D();
    Matrix6D(const Matrix6D& other);
    Matrix6D(const Vector6D& diag);
    Matrix6D(const double* arr);
    Matrix6D(const Matrix3D &upperLeft, const Matrix3D &upperRight, const Matrix3D &lowerLeft, const Matrix3D &lowerRight);

    Vector6D getColumn(const int &j) const;
    Vector6D getRow(const int &i) const;
    Vector6D diag() const;
    Matrix6D transpose() const;
    Matrix6D invert() const;
    void setColumn(const int &j, const Vector6D& column);
    void setRow(const int &i, const Vector6D& row);
    Matrix6D scale(const double &factor) const;
    Matrix6D mAdd(const Matrix6D& other) const;
    Matrix6D mSub(const Matrix6D& other) const;
    Matrix6D mMult(const Matrix6D& other) const;

    Matrix3D upperLeft() const;
    Matrix3D upperRight() const;
    Matrix3D lowerLeft() const;
    Matrix3D lowerRight() const;
};


/*******************        functions       **********************/
Matrix6D dyadic(const Vector6D& left, const Vector6D& right);
bool ludcmp(Matrix6D &a, Vector6D &indx, double &d);
void lubksb(Matrix6D &a, Vector6D &indx, Vector6D &b);

/*******************        operators       **********************/
inline Matrix6D operator*(const double &left,   const Matrix6D& right) { return right.scale(left); }
inline Matrix6D operator*(const Matrix6D& left, const double &right) { return left.scale(right); }
inline Matrix6D operator/(const Matrix6D &left, const double &right) { return left.scale(1.0/right); }
inline Matrix6D operator+(const Matrix6D& left, const Matrix6D& right) { return left.mAdd(right); }
inline Matrix6D operator-(const Matrix6D& left, const Matrix6D& right) { return left.mSub(right); }
inline Matrix6D operator*(const Matrix6D& left, const Matrix6D& right) { return left.mMult(right); }
inline Matrix6D operator*(const Vector6D &left, const Vector6D &right) { return dyadic(left, right); }
inline Vector6D operator*(const Matrix6D& left, const Vector6D& right) { return right.matVecMult(left); }


#ifndef NO_RODOS_NAMESPACE
}
#endif

