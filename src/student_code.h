#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

#include "halfEdgeMesh.h"

using namespace std;

namespace CMU462 {

   class MeshResampler{

      public:

         MeshResampler(){};
         ~MeshResampler(){}

         void upsample  ( HalfedgeMesh& mesh );
         void downsample( HalfedgeMesh& mesh );
         void resample  ( HalfedgeMesh& mesh );

         int degree( VertexIter v );
   };
}

#endif // STUDENT_CODE_H
