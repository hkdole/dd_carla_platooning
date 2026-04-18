// file: ~/omnet6_ws/plexe/src/org/car2x/plexe/vis/SumoNetOverlay.cc
#include <omnetpp.h>
#include <sstream>
#include <vector>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>                      // <-- add
using namespace omnetpp;

class SumoNetOverlay : public cSimpleModule {
  protected:
    cGroupFigure *group = nullptr;
    cGroupFigure *debugGroup = nullptr;

    double scale=1.0, rot=0.0, offx=0.0, offy=0.0; bool flipX=false, flipY=true, aa=true;
    bool autoFit=false, preserveAspect=true, unitLock=true;
    double fitW=395.0, fitH=330.0, fitM=0.0;
    double netMinX=0, netMinY=0, netMaxX=0, netMaxY=0;
    bool drawDebug=true;

    struct Extents { double minx, miny, maxx, maxy; };
    static std::vector<std::pair<double,double>> parseShape(const char* s){
        std::vector<std::pair<double,double>> pts; std::stringstream ss(s); std::string tok;
        while (ss >> tok){ auto c=tok.find(','); if(c!=std::string::npos)
            pts.emplace_back(std::stod(tok.substr(0,c)), std::stod(tok.substr(c+1))); }
        return pts;
    }
    static bool parseDoubles(const char* s, std::vector<double>& out){
        if(!s) return false; std::stringstream ss(s); std::string tok; out.clear();
        while (std::getline(ss,tok,',')) if(!tok.empty()) out.push_back(std::stod(tok));
        return !out.empty();
    }
    std::pair<double,double> xf(double x,double y) const{
        if (flipX) x = -x;
        if (flipY) y = -y;
        const double xr =  x*std::cos(rot) - y*std::sin(rot);
        const double yr =  x*std::sin(rot) + y*std::cos(rot);
        return { offx + scale*xr, offy + scale*yr };
    }
    static Extents bbox(const std::vector<std::vector<std::pair<double,double>>>& lines){
        Extents b{+std::numeric_limits<double>::infinity(),
                  +std::numeric_limits<double>::infinity(),
                  -std::numeric_limits<double>::infinity(),
                  -std::numeric_limits<double>::infinity()};
        for (auto &ln: lines) for (auto &p: ln){
            b.minx = std::min(b.minx, p.first);  b.maxx = std::max(b.maxx, p.first);
            b.miny = std::min(b.miny, p.second); b.maxy = std::max(b.maxy, p.second);
        }
        return b;
    }

    // keep one group per canvas (module instance)
    static cGroupFigure* ensureGroup(cCanvas* cv, const char* name, int z){
        static std::map<cCanvas*, cGroupFigure*> cache;
        auto it = cache.find(cv);
        if (it != cache.end()) return it->second;
        cGroupFigure* g = new cGroupFigure(name);
        g->setZIndex(z);
        cv->addFigure(g);
        cache[cv] = g;
        return g;
    }
    static void dot(cGroupFigure* g, double x,double y, const cFigure::Color& col, double r=3, double alpha=0.9){
        cOvalFigure* f = new cOvalFigure(nullptr);
        f->setBounds(cFigure::Rectangle(x-r,y-r, 2*r,2*r));
        f->setFilled(true);
        f->setFillColor(col);
        f->setLineColor(col);
        f->setFillOpacity(alpha);
        f->setLineOpacity(alpha);
        g->addFigure(f);
    }
    static void line(cGroupFigure* g, double x1,double y1,double x2,double y2, const cFigure::Color& col, double w=1.5){
        cLineFigure* l = new cLineFigure(nullptr);
        l->setStart(cFigure::Point(x1,y1)); l->setEnd(cFigure::Point(x2,y2));
        l->setLineColor(col); l->setLineWidth(w);
        g->addFigure(l);
    }
    static void rect(cGroupFigure* g, double x,double y,double w,double h, const cFigure::Color& col, double alpha=0.15){
        cRectangleFigure* r = new cRectangleFigure(nullptr);
        r->setBounds(cFigure::Rectangle(x,y,w,h));
        r->setFilled(true);
        r->setFillColor(col);
        r->setFillOpacity(alpha);
        r->setLineColor(col);
        r->setLineWidth(1);
        g->addFigure(r);
    }


  protected:
    virtual void initialize() override {
        scale = par("scale").doubleValue();
        rot   = par("rotDeg").doubleValue() * M_PI/180.0;
        offx  = par("offsetX").doubleValue();
        offy  = par("offsetY").doubleValue();
        flipX = par("flipX").boolValue();
        flipY = par("flipY").boolValue();
        aa    = par("antiAlias").boolValue();

        autoFit        = par("autoFit").boolValue();
        unitLock       = par("unitLock").boolValue();
        preserveAspect = par("preserveAspect").boolValue();
        fitW = par("fitWidth").doubleValue();
        fitH = par("fitHeight").doubleValue();
        fitM = par("fitMargin").doubleValue();

        if (hasPar("drawDebug")) drawDebug = par("drawDebug").boolValue();

        cXMLElement *root = par("netXml").xmlValue();
        if (!root) throw cRuntimeError("SumoNetOverlay: netXml (xmldoc) not set");

        // Parse <location> (netOffset, convBoundary)
        if (auto *loc = root->getFirstChildWithAttribute("location", nullptr, nullptr)){
            std::vector<double> v;
            if (parseDoubles(loc->getAttribute("netOffset"), v) && v.size()>=2){
                par("sumoNetOffsetX").setDoubleValue(v[0]);
                par("sumoNetOffsetY").setDoubleValue(v[1]);
            }
            if (parseDoubles(loc->getAttribute("convBoundary"), v) && v.size()>=4){
                netMinX=v[0]; netMinY=v[1]; netMaxX=v[2]; netMaxY=v[3];
                par("netMinX").setDoubleValue(netMinX);
                par("netMinY").setDoubleValue(netMinY);
                par("netMaxX").setDoubleValue(netMaxX);
                par("netMaxY").setDoubleValue(netMaxY);
            }
        }

        // Collect lanes
        std::vector<std::vector<std::pair<double,double>>> lines;
        for (auto *edge=root->getFirstChild(); edge; edge=edge->getNextSibling()){
            if (std::strcmp(edge->getTagName(),"edge")!=0) continue;
            for (auto *lane=edge->getFirstChild(); lane; lane=lane->getNextSibling()){
                if (std::strcmp(lane->getTagName(),"lane")!=0) continue;
                const char* shp = lane->getAttribute("shape"); if(!shp) continue;
                auto pts = parseShape(shp); if(pts.size()<2) continue;
                lines.emplace_back(std::move(pts));
            }
        }
        if (lines.empty()) return;

        const auto b = bbox(lines);  // SUMO meters

        // Choose transform
        if (unitLock){
            scale = 1.0;
            offx  = -(flipX ? -b.maxx : b.minx);
            offy  = -(flipY ? -b.maxy : b.miny);
            par("scale").setDoubleValue(scale);
            par("offsetX").setDoubleValue(offx);
            par("offsetY").setDoubleValue(offy);
            par("rotDeg").setDoubleValue(rot * 180.0 / M_PI);
        } else if (autoFit){
            const double w = (b.maxx - b.minx);
            const double h = (b.maxy - b.miny);
            const double sx = (fitW - 2*fitM) / std::max(1e-9, w);
            const double sy = (fitH - 2*fitM) / std::max(1e-9, h);
            const double s  = preserveAspect ? std::min(sx, sy) : sx;
            scale = s;
            offx  = -(flipX ? -b.maxx : b.minx) * scale + fitM;
            offy  = -(flipY ? -b.maxy : b.miny) * scale + fitM;
            par("scale").setDoubleValue(scale);
            par("offsetX").setDoubleValue(offx);
            par("offsetY").setDoubleValue(offy);
            par("rotDeg").setDoubleValue(rot * 180.0 / M_PI);
        }

        // Draw lanes
        cCanvas *cv = getParentModule()->getCanvas();
        group = ensureGroup(cv, "sumoNet", -100);
        for (auto &pts: lines){
            cPolylineFigure* pl = new cPolylineFigure(nullptr);
            pl->setLineWidth(1); pl->setLineOpacity(0.9);
            pl->setLineColor(cFigure::Color(128,128,128)); pl->setSmooth(aa);
            for (auto &p: pts){ auto q=xf(p.first,p.second); pl->addPoint({q.first,q.second}); }
            group->addFigure(pl);
        }

        // --- DEBUG: draw origin, axes, and convBoundary ---
        if (drawDebug){
            cCanvas *cv = getParentModule()->getCanvas();
            debugGroup = ensureGroup(cv, "sumoNetDebug", -90);

            std::pair<double,double> O = xf(0,0);
            dot(debugGroup, O.first, O.second, cFigure::Color(255,0,255), 3.5, 1.0);

            std::pair<double,double> Xaxis = xf(30,0);
            std::pair<double,double> Yaxis = xf(0,30);
            line(debugGroup, O.first, O.second, Xaxis.first, Xaxis.second, cFigure::Color(255,0,0),   2.0);
            line(debugGroup, O.first, O.second, Yaxis.first, Yaxis.second, cFigure::Color(0,200,0),   2.0);

            std::pair<double,double> C1 = xf(netMinX, netMinY);
            std::pair<double,double> C2 = xf(netMaxX, netMaxY);
            const double rx = std::min(C1.first, C2.first);
            const double ry = std::min(C1.second, C2.second);
            const double rw = std::fabs(C2.first - C1.first);
            const double rh = std::fabs(C2.second - C1.second);
            rect(debugGroup, rx, ry, rw, rh, cFigure::Color(0,200,200), 0.12);
            dot(debugGroup, C1.first, C1.second, cFigure::Color(0,255,255), 3.0, 1.0);
            dot(debugGroup, C2.first, C2.second, cFigure::Color(0,255,255), 3.0, 1.0);
        }
    }
};
Define_Module(SumoNetOverlay);