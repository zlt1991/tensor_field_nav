#include <octomap_tensor_field/MyOcTree.h>

namespace octomap {


  // node implementation  --------------------------------------
  std::ostream& MyOcTreeNode::writeValue (std::ostream &s) const {
    // 1 bit for each children; 0: empty, 1: allocated
    std::bitset<8> children;
    for (unsigned int i=0; i<8; i++) {
      if (childExists(i)) children[i] = 1;
      else                children[i] = 0;
    }
    char children_char = (char) children.to_ulong();

    // write node data
    s.write((const char*) &value, sizeof(value)); // occupancy
    s.write((const char*) &color, sizeof(Color)); // color
    s.write((const char*) &sqrtRecipStd, sizeof(sqrtRecipStd)); // sqrtRecipStd
    s.write((char*)&children_char, sizeof(char)); // child existence

    // write existing children
    for (unsigned int i=0; i<8; ++i)
      if (children[i] == 1) this->getChild(i)->writeValue(s);
    return s;
  }

  std::istream& MyOcTreeNode::readValue (std::istream &s) {
    // read node data
    char children_char;
    s.read((char*) &value, sizeof(value)); // occupancy
    s.read((char*) &color, sizeof(Color)); // color
    s.read((char*) &sqrtRecipStd, sizeof(sqrtRecipStd)); // sqrtRecipStd
    s.read((char*)&children_char, sizeof(char)); // child existence

    // read existing children
    std::bitset<8> children ((unsigned long long) children_char);
    for (unsigned int i=0; i<8; i++) {
      if (children[i] == 1){
        createChild(i);
        getChild(i)->readValue(s);
      }
    }
    return s;
  }

  MyOcTreeNode::Color MyOcTreeNode::getAverageChildColor() const {
    int mr(0), mg(0), mb(0);
    int c(0);
    for (int i=0; i<8; i++) {
      if (childExists(i) && getChild(i)->isColorSet()) {
        mr += getChild(i)->getColor().r;
        mg += getChild(i)->getColor().g;
        mb += getChild(i)->getColor().b;
        ++c;
      }
    }
    if (c) {
      mr /= c;
      mg /= c;
      mb /= c;
      return Color((unsigned char) mr, (unsigned char) mg, (unsigned char) mb);
    }
    else { // no child had a color other than white
      return Color(255, 255, 255);
    }
  }
  double MyOcTreeNode::getMeanChildSqrtRecipStd() const{
    double mean = 0;
    double c = 0;
    if (hasChildren()){
      for (unsigned int i=0; i<8; i++) {
        if (childExists(i)) {
          mean += getChild(i)->getSqrtRecipStd(); // TODO check if works generally
          ++c;
        }
      }
    }

    if (c > 0)
      mean /= (double) c;

    return mean;
  }

  double MyOcTreeNode::getMaxChildSqrtRecipStd() const{
    double max = -std::numeric_limits<float>::max();

    if (hasChildren()){
      for (unsigned int i=0; i<8; i++) {
        if (childExists(i)) {
          double l = getChild(i)->getSqrtRecipStd(); // TODO check if works generally
          if (l > max)
            max = l;
        }
      }
    }
    return max;
  }


  void MyOcTreeNode::updateColorChildren() {
    color = getAverageChildColor();
  }
  void MyOcTreeNode::updateSqrtRecipStdChildren() {
    sqrtRecipStd = getMaxChildSqrtRecipStd();
  }

  // pruning =============

  bool MyOcTreeNode::pruneNode() {
    // checks for equal occupancy only, color ignored
    if (!this->collapsible()) return false;
    // set occupancy value
    setLogOdds(getChild(0)->getLogOdds());
    // set color to average color
    if (isColorSet()) color = getAverageChildColor();
    sqrtRecipStd=getMaxChildSqrtRecipStd();
    // delete children
    for (unsigned int i=0;i<8;i++) {
      delete children[i];
    }
    delete[] children;
    children = NULL;
    return true;
  }

  void MyOcTreeNode::expandNode() {
    assert(!hasChildren());
    for (unsigned int k=0; k<8; k++) {
      createChild(k);
      children[k]->setValue(value);
      getChild(k)->setColor(color);
      getChild(k)->setSqrtRecipStd(sqrtRecipStd);
    }
  }
  void MyOcTreeNode::addSqrtRecipStd(const double& sqrtRecipStd_update){
        sqrtRecipStd+=sqrtRecipStd_update;
  }


  // tree implementation  --------------------------------------

  MyOcTreeNode* MyOcTree::setNodeColor(const OcTreeKey& key,
                                             const unsigned char& r,
                                             const unsigned char& g,
                                             const unsigned char& b) {
    MyOcTreeNode* n = search (key);
    if (n != 0) {
      n->setColor(r, g, b);
    }
    return n;
  }

  MyOcTreeNode* MyOcTree::setNodeSqrtRecipStd(const OcTreeKey& key,const double &sqrtRecipStd_  ) {
    MyOcTreeNode* n = search (key);
    if (n != 0) {
      n->setSqrtRecipStd(sqrtRecipStd_);
    }
    return n;
  }
  MyOcTreeNode* MyOcTree::setNodeCurDist(const OcTreeKey& key,const double &dist  ) {
    MyOcTreeNode* n = search (key);
    if (n != 0) {
      n->setCurDist(dist);
    }
    return n;
  }

double MyOcTree::getNodeCurDist(const OcTreeKey& key){
    MyOcTreeNode* n = search (key);
    if (n != 0) {
      return n->getCurDist();
    }
    return -1.0;
}
double MyOcTree::getNodeSqrtRecipStdt(const OcTreeKey& key){
    MyOcTreeNode* n = search (key);
    if (n != 0) {
      return n->getSqrtRecipStd();
    }
    return -1.0;
}

int MyOcTree::getNodeVisCount(const OcTreeKey& key){
    MyOcTreeNode* n = search (key);
    if (n != 0) {
      return n->getVisCount();
    }
    return -1;
}


  MyOcTreeNode* MyOcTree::averageNodeColor(const OcTreeKey& key,
                                                 const unsigned char& r,
                                                 const unsigned char& g,
                                                 const unsigned char& b) {
    MyOcTreeNode* n = search (key);
    if (n != 0) {
      if (n->isColorSet()) {
        MyOcTreeNode::Color prev_color = n->getColor();
        n->setColor((prev_color.r + r)/2, (prev_color.g + g)/2, (prev_color.b + b)/2);
      }
      else {
        n->setColor(r, g, b);
      }
    }
    return n;
  }

  MyOcTreeNode* MyOcTree::integrateNodeColor(const OcTreeKey& key,
                                                   const unsigned char& r,
                                                   const unsigned char& g,
                                                   const unsigned char& b) {
    MyOcTreeNode* n = search (key);
    if (n != 0) {
      if (n->isColorSet()) {
        MyOcTreeNode::Color prev_color = n->getColor();
        double node_prob = n->getOccupancy();
        unsigned char new_r = (unsigned char) ((double) prev_color.r * node_prob
                                               +  (double) r * (0.99-node_prob));
        unsigned char new_g = (unsigned char) ((double) prev_color.g * node_prob
                                               +  (double) g * (0.99-node_prob));
        unsigned char new_b = (unsigned char) ((double) prev_color.b * node_prob
                                               +  (double) b * (0.99-node_prob));
        n->setColor(new_r, new_g, new_b);
      }
      else {
        n->setColor(r, g, b);
      }
    }
    return n;
  }

   void MyOcTree::updateNodeSqrtRecipStd(const OcTreeKey& key,const double &sqrtRecipStd_update ){
       MyOcTreeNode* n = search (key);
       if (n != 0) {
         n->addSqrtRecipStd(sqrtRecipStd_update);
       }
   }

   void MyOcTree::updateNodeVisCount(const OcTreeKey& key){
       MyOcTreeNode* n = search (key);
       if (n != 0) {
         int curVisCount=n->getVisCount();
         if (curVisCount==claming_visCount_thres)
             return;
         n->setVisCount(curVisCount+1);
       }
   }

  void MyOcTree::updateInnerOccupancy() {
    if(this->root)
    this->updateInnerOccupancyRecurs(this->root, 0);
  }

  void MyOcTree::updateInnerOccupancyRecurs(MyOcTreeNode* node, unsigned int depth) {
    // only recurse and update for inner nodes:
    if (node->hasChildren()){
      // return early for last level:
      if (depth < this->tree_depth){
        for (unsigned int i=0; i<8; i++) {
          if (node->childExists(i)) {
            updateInnerOccupancyRecurs(node->getChild(i), depth+1);
          }
        }
      }
      node->updateOccupancyChildren();
      node->updateColorChildren();
      node->updateSqrtRecipStdChildren();
    }
  }
  void MyOcTree::groundColorMix(double* color, double x, double min, double max)
  {
     /*
      * Red = 0
      * Green = 1
      * Blue = 2
      */
      double posSlope = (max-min)/60;
      double negSlope = (min-max)/60;

      if( x < 60 )
      {
          color[0] = max;
          color[1] = posSlope*x+min;
          color[2] = min;
          return;
      }
      else if ( x < 120 )
      {
          color[0] = negSlope*x+2*max+min;
          color[1] = max;
          color[2] = min;
          return;
      }
      else if ( x < 180  )
      {
          color[0] = min;
          color[1] = max;
          color[2] = posSlope*x-2*max+min;
          return;
      }
      else if ( x < 240  )
      {
          color[0] = min;
          color[1] = negSlope*x+4*max+min;
          color[2] = max;
          return;
      }
      else if ( x < 300  )
      {
          color[0] = posSlope*x-4*max+min;
          color[1] = min;
          color[2] = max;
          return;
      }
      else
      {
          color[0] = max;
          color[1] = min;
          color[2] = negSlope*x+6*max;
          return;
      }
  }


  void MyOcTree::writeColorHistogram(std::string filename) {

#ifdef _MSC_VER
    fprintf(stderr, "The color histogram uses gnuplot, this is not supported under windows.\n");
#else
    // build RGB histogram
    std::vector<int> histogram_r (256,0);
    std::vector<int> histogram_g (256,0);
    std::vector<int> histogram_b (256,0);
    for(MyOcTree::tree_iterator it = this->begin_tree(),
          end=this->end_tree(); it!= end; ++it) {
      if (!it.isLeaf() || !this->isNodeOccupied(*it)) continue;
      MyOcTreeNode::Color& c = it->getColor();
      ++histogram_r[c.r];
      ++histogram_g[c.g];
      ++histogram_b[c.b];
    }
    // plot data
    FILE *gui = popen("gnuplot ", "w");
    fprintf(gui, "set term postscript eps enhanced color\n");
    fprintf(gui, "set output \"%s\"\n", filename.c_str());
    fprintf(gui, "plot [-1:256] ");
    fprintf(gui,"'-' w filledcurve lt 1 lc 1 tit \"r\",");
    fprintf(gui, "'-' w filledcurve lt 1 lc 2 tit \"g\",");
    fprintf(gui, "'-' w filledcurve lt 1 lc 3 tit \"b\",");
    fprintf(gui, "'-' w l lt 1 lc 1 tit \"\",");
    fprintf(gui, "'-' w l lt 1 lc 2 tit \"\",");
    fprintf(gui, "'-' w l lt 1 lc 3 tit \"\"\n");

    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_r[i]);
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_g[i]);
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_b[i]);
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_r[i]);
    fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_g[i]);
    fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_b[i]);
    fprintf(gui, "e\n");
    fflush(gui);
#endif
  }

  std::ostream& operator<<(std::ostream& out, MyOcTreeNode::Color const& c) {
    return out << '(' << (unsigned int)c.r << ' ' << (unsigned int)c.g << ' ' << (unsigned int)c.b << ')';
  }


  MyOcTree::StaticMemberInitializer MyOcTree::MyOcTreeMemberInit;

} // end namespace
