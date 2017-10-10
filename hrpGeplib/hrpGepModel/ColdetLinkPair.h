/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef HRPGEPMODEL_COLDET_LINK_PAIR_H_INCLUDED
#define HRPGEPMODEL_COLDET_LINK_PAIR_H_INCLUDED

#include <hrpGepCollision/ColdetModelPair.h>
#include "Link.h"
#include "Config.h"

namespace hrpGep {
    
    class Link;
    
    class HRPGEPMODEL_API ColdetLinkPair : public ColdetModelPair
    {
      public:
        ColdetLinkPair(Link* link1, Link* link2, double tolerance=0);
        
        ColdetLinkPair(const ColdetLinkPair& org);
        
        virtual ~ColdetLinkPair();
        
        void updatePositions();
        
        Link* link(int index);
        
      protected:
        Link* links[2];
        
      private:
    };
    
    typedef boost::intrusive_ptr<ColdetLinkPair> ColdetLinkPairPtr;
}

#endif
