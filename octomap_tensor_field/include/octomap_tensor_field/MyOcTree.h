#ifndef MYOCTREE_H
#define MYOCTREE_H


#include <iostream>
#include "octomap/OccupancyOcTreeBase.h"
//#include "octomap_server/MyOcTreeNode.h"

namespace octomap {

  // node definition
  class MyOcTreeNode : public OcTreeNode {
  public:

    class Color {
    public:
    Color() : r(255), g(255), b(255) {}
    Color(unsigned char _r, unsigned char _g, unsigned char _b)
      : r(_r), g(_g), b(_b) {}
      inline bool operator== (const Color &other) const {
        return (r==other.r && g==other.g && b==other.b);
      }
      inline bool operator!= (const Color &other) const {
        return (r!=other.r || g!=other.g || b!=other.b);
      }
      unsigned char r, g, b;
    };

  public:
    MyOcTreeNode() : OcTreeNode() {sqrtRecipStd=1.0/pow(0.05,2); visCount=0;}

    MyOcTreeNode(const MyOcTreeNode& rhs) : OcTreeNode(rhs), color(rhs.color),sqrtRecipStd(rhs.sqrtRecipStd),curDist(rhs.curDist) {}

    bool operator==(const MyOcTreeNode& rhs) const{
      return (rhs.value == value && rhs.color == color&& rhs.sqrtRecipStd==sqrtRecipStd);
    }

    // children
    inline MyOcTreeNode* getChild(unsigned int i) {
      return static_cast<MyOcTreeNode*> (OcTreeNode::getChild(i));
    }
    inline const MyOcTreeNode* getChild(unsigned int i) const {
      return static_cast<const MyOcTreeNode*> (OcTreeNode::getChild(i));
    }

    bool createChild(unsigned int i) {
      if (children == NULL) allocChildren();
      children[i] = new MyOcTreeNode();
      return true;
    }

    bool pruneNode();
    void expandNode();

    inline Color getColor() const { return color; }
    inline void  setColor(Color c) {this->color = c; }
    inline void  setColor(unsigned char r, unsigned char g, unsigned char b) {
      this->color = Color(r,g,b);
    }

    Color& getColor() { return color; }

    // has any color been integrated? (pure white is very unlikely...)
    inline bool isColorSet() const {
      return ((color.r != 255) || (color.g != 255) || (color.b != 255));
    }

    void updateColorChildren();
    void updateSqrtRecipStdChildren();

    MyOcTreeNode::Color getAverageChildColor() const;
    double getMaxChildSqrtRecipStd() const;
    double getMeanChildSqrtRecipStd() const;
    // file I/O
    std::istream& readValue (std::istream &s);
    std::ostream& writeValue(std::ostream &s) const;


    //
    inline double getSqrtRecipStd() const { return sqrtRecipStd; }
    inline double setSqrtRecipStd(double sqrtRecipStd_) { this->sqrtRecipStd=sqrtRecipStd_; }
    inline double getCurDist() const { return curDist; }
    inline double setCurDist(double dist){ this->curDist=dist; }
    inline double getVisCount() const { return curDist; }
    inline double setVisCount(int visCount_){ this->visCount=visCount_; }

    void addSqrtRecipStd(const double& sqrtRecipStd_update);


    //

  protected:
    Color color;
    double sqrtRecipStd;
    float curDist;
    int visCount;
  };


  // tree definition
  class MyOcTree : public OccupancyOcTreeBase <MyOcTreeNode> {

  public:
    /// Default constructor, sets resolution of leafs
    MyOcTree(double resolution) : OccupancyOcTreeBase<MyOcTreeNode>(resolution) {claming_visCount_thres=360;}

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    MyOcTree* create() const {return new MyOcTree(resolution); }

    std::string getTreeType() const {return "ColorTree";}

    void setClampingVisCountThres(int max_visCount_){claming_visCount_thres=max_visCount_;}


    // set node color at given key or coordinate. Replaces previous color.
    MyOcTreeNode* setNodeColor(const OcTreeKey& key, const unsigned char& r,
                                 const unsigned char& g, const unsigned char& b);

    MyOcTreeNode* setNodeColor(const float& x, const float& y,
                                 const float& z, const unsigned char& r,
                                 const unsigned char& g, const unsigned char& b) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return setNodeColor(key,r,g,b);
    }
    MyOcTreeNode* setNodeSqrtRecipStd(const OcTreeKey& key,const double &sqrtRecipStd_ );

    MyOcTreeNode* setNodeSqrtRecipStd(const float& x, const float& y,
                                 const float& z,const double &sqrtRecipStd_) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return setNodeSqrtRecipStd(key,sqrtRecipStd_);
    }
    MyOcTreeNode* setNodeCurDist(const OcTreeKey& key,const double &dist  );

    MyOcTreeNode* setNodeCurDist(const float& x, const float& y,
                                 const float& z,const double &dist) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return setNodeCurDist(key,dist);
    }

    double getNodeCurDist(const OcTreeKey& key);

    double getNodeCurDist(const float& x, const float& y,
                          const float& z){
        OcTreeKey key;
        if (!this->coordToKeyChecked(point3d(x,y,z), key)) return -1.0;
        return getNodeCurDist(key);
    }
    double getNodeSqrtRecipStdt(const OcTreeKey& key);

    double getNodeSqrtRecipStdt(const float& x, const float& y,
                          const float& z){
        OcTreeKey key;
        if (!this->coordToKeyChecked(point3d(x,y,z), key)) return -1.0;
        return getNodeSqrtRecipStdt(key);
    }

    int getNodeVisCount(const OcTreeKey& key);

    int getNodeVisCount(const float& x, const float& y,
                          const float& z){
        OcTreeKey key;
        if (!this->coordToKeyChecked(point3d(x,y,z), key)) return -1;
        return getNodeVisCount(key);
    }

    // integrate color measurement at given key or coordinate. Average with previous color
    MyOcTreeNode* averageNodeColor(const OcTreeKey& key, const unsigned char& r,
                                  const unsigned char& g, const unsigned char& b);

    MyOcTreeNode* averageNodeColor(const float& x, const float& y,
                                      const float& z, const unsigned char& r,
                                      const unsigned char& g, const unsigned char& b) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return averageNodeColor(key,r,g,b);
    }

    // integrate color measurement at given key or coordinate. Average with previous color
    MyOcTreeNode* integrateNodeColor(const OcTreeKey& key, const unsigned char& r,
                                  const unsigned char& g, const unsigned char& b);

    MyOcTreeNode* integrateNodeColor(const float& x, const float& y,
                                      const float& z, const unsigned char& r,
                                      const unsigned char& g, const unsigned char& b) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return integrateNodeColor(key,r,g,b);
    }

    // update inner nodes, sets color to average child color
    void updateInnerOccupancy();

    void updateNodeSqrtRecipStd(const OcTreeKey& key,const double &sqrtRecipStd_update );
    void updateNodeSqrtRecipStd(const float& x, const float& y,
                                 const float& z,const double &sqrtRecipStd_update) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return;
      updateNodeSqrtRecipStd(key,sqrtRecipStd_update);
    }

    void updateNodeVisCount(const OcTreeKey& key);
    void updateNodeVisCount(const float& x, const float& y,
                                 const float& z) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return;
      updateNodeVisCount(key);
    }



    inline double getModelNoiseStd(double dist){
        return 0.0012+0.0019*pow((dist-0.4),2);
    }
    inline double CalCurSqrtRecipStd(double dist){
        return 1/pow(getModelNoiseStd(dist),2);
    }
    void groundColorMix(double* color, double x, double min, double max);
    // uses gnuplot to plot a RGB histogram in EPS format
    void writeColorHistogram(std::string filename);

  protected:
    void updateInnerOccupancyRecurs(MyOcTreeNode* node, unsigned int depth);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once
     */
    class StaticMemberInitializer{
       public:
         StaticMemberInitializer() {
           MyOcTree* tree = new MyOcTree(0.1);
           AbstractOcTree::registerTreeType(tree);
         }
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer MyOcTreeMemberInit;
    int claming_visCount_thres;

  };

  //! user friendly output in format (r g b)
  std::ostream& operator<<(std::ostream& out, MyOcTreeNode::Color const& c);

} // end namespace

#endif
