/*
 * Student solution for CMU 15-462 Project 2 (MeshEdit)
 *
 * Implemented by Xiaoshan Lu on ____.
 *
 */

#include "student_code.h"
#include "mutablePriorityQueue.h"

namespace CMU462
{
  MutablePriorityQueue<EdgeRecord> erQueue;

   VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
   {
      // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
      // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      if(e0->halfedge()->face()->isBoundary()||e0->halfedge()->twin()->face()->isBoundary()){
        std::cerr << "Cannot split a boundary." << std::endl;
        // throw EdgeEditException(20);
        return VertexIter();
      }

      HalfedgeIter h[12];

      h[0] = e0->halfedge();
      h[1] = h[0]->next();
      h[2] = h[1]->next();
      h[3] = h[0]->twin();
      h[4] = h[3]->next();
      h[5] = h[4]->next();
      // new
      for(int i=6;i<12;i++){
        h[i] = newHalfedge();
      }

      VertexIter v[5];
      v[0] = h[0]->vertex();
      v[1] = h[1]->vertex();
      v[2] = h[2]->vertex();
      v[3] = h[5]->vertex();
      // new
      v[4] = newVertex();
      v[4]->isNew = true;
      // assign
      v[4]->halfedge() = h[0];
      v[4]->position = (v[0]->position + v[1]->position)/2;

      FaceIter f[4];
      f[0] = h[0]->face();
      f[1] = h[3]->face();
      // new
      for(int i = 2;i<4;i++){
        f[i] = newFace();
      }
      // assign
      f[2]->halfedge() = h[2];
      f[3]->halfedge() = h[5];

      EdgeIter e[4];
      e[0] = e0;                e[0]->isNew = false;
      for(int i=1;i<4;i++){
        e[i] = newEdge();
      }
      // assign
      e[1]->halfedge() = h[3];  e[1]->isNew = false;
      e[2]->halfedge() = h[6];  e[2]->isNew = true;
      e[3]->halfedge() = h[11]; e[3]->isNew = true;

      //                    next        ,twin         ,vertex         ,edge         ,face
      h[0 ]-> setNeighbors(h[0]->next() ,h[9]         ,v[4]           ,h[0]->edge() ,f[0]   );
      h[1 ]-> setNeighbors(h[6]         ,h[1]->twin() ,h[1]->vertex() ,h[1]->edge() ,f[0]   );
      h[2 ]-> setNeighbors(h[8]         ,h[2]->twin() ,h[2]->vertex() ,h[2]->edge() ,f[2]   );
      h[3 ]-> setNeighbors(h[3]->next() ,h[8]         ,v[4]           ,e[1]         ,f[1]   );
      h[4 ]-> setNeighbors(h[11]        ,h[4]->twin() ,h[4]->vertex() ,h[4]->edge() ,f[1]   );
      h[5 ]-> setNeighbors(h[9]         ,h[5]->twin() ,h[5]->vertex() ,h[5]->edge() ,f[3]   );
      h[6 ]-> setNeighbors(h[0]         ,h[7]         ,v[2]           ,e[2]         ,f[0]   );
      h[7 ]-> setNeighbors(h[2]         ,h[6]         ,v[4]           ,e[2]         ,f[2]   );
      h[8 ]-> setNeighbors(h[7]         ,h[3]         ,v[0]           ,e[1]         ,f[2]   );
      h[9 ]-> setNeighbors(h[10]        ,h[0]         ,v[1]           ,e[0]         ,f[3]   );
      h[10]-> setNeighbors(h[5]         ,h[11]        ,v[4]           ,e[3]         ,f[3]   );
      h[11]-> setNeighbors(h[3]         ,h[10]        ,v[3]           ,e[3]         ,f[1]   );

      f[0]->halfedge() = h[1];
      f[1]->halfedge() = h[4];
      e0->halfedge() = h[0];

      v[0]->halfedge() = h[4];
      v[1]->halfedge() = h[1];
      v[2]->halfedge() = h[2];
      v[3]->halfedge() = h[5];

			return v[4];
	 }

   VertexIter HalfedgeMesh::collapseEdge( EdgeIter e0 )
   {
     // TODO This method should collapse the given edge and return an iterator to the new vertex created by the collapse.
     // check
     if(e0->halfedge()->face()->isBoundary()||e0->halfedge()->twin()->face()->isBoundary()
     ||e0->halfedge()->next()->twin()->face()->isBoundary()||e0->halfedge()->next()->next()->twin()->face()->isBoundary()
     ||e0->halfedge()->twin()->next()->twin()->face()->isBoundary()||e0->halfedge()->twin()->next()->next()->twin()->face()->isBoundary()){
      //  throw EdgeEditException(31);
        std::cerr << "Cannot collapse a boundary." << std::endl;
        return VertexIter();
     }

     HalfedgeIter h[10];

     h[0] = e0->halfedge();
     h[1] = h[0]->next();
     h[2] = h[1]->next();
     h[3] = h[0]->twin();
     h[4] = h[3]->next();
     h[5] = h[4]->next();
     h[6] = h[1]->twin();
     h[7] = h[2]->twin();
     h[8] = h[4]->twin();
     h[9] = h[5]->twin();
     // check
     if(h[6]->next()==h[9]&&h[8]->next()==h[7]&&h[7]->next()==h[9]->next()->twin()){
       std::cerr << "Cannot collapse an edge of a tetrahedral." << std::endl;
       return VertexIter();
     }


     VertexIter v[4];
     v[0] = h[0]->vertex();
     v[1] = h[1]->vertex();
     v[2] = h[2]->vertex();
     v[3] = h[5]->vertex();
     // check
    //  for(int i=0;i<4;i++){
    //    if(v[i]->degree()<=3){
    //        std::cerr << "Cannot collapse this edge." << std::endl;
    //       //  throw EdgeEditException(30);
    //        return v[0];
    //    }
    //  }

     v[0]->position = (v[0]->position + v[1]->position)/2;

     EdgeIter e[5];
     e[0] = e0;
     e[1] = h[1]->edge();
     e[2] = h[2]->edge();
     e[3] = h[4]->edge();
     e[4] = h[5]->edge();
     for(int i=0;i<5;i++){
       erQueue.remove(e[i]);
     }

     FaceIter f[2];
     f[0] = h[0]->face();
     f[1] = h[3]->face();


     // setNeighbors
     h[6]->setNeighbors(h[6]->next() ,h[7]     ,h[6]->vertex() ,e[1]      ,h[6]->face());
     h[7]->setNeighbors(h[7]->next() ,h[6]     ,v[0]           ,e[1]      ,h[7]->face());
     h[8]->setNeighbors(h[8]->next() ,h[9]     ,h[8]->vertex() ,e[4]      ,h[8]->face());
     h[9]->setNeighbors(h[9]->next() ,h[8]     ,v[0]           ,e[4]      ,h[9]->face());
     v[0]->halfedge() = h[9];
     v[2]->halfedge() = h[6];
     v[3]->halfedge() = h[8];
     e[1]->halfedge() = h[6];
     e[4]->halfedge() = h[9];

     // reassign every neighbors
     HalfedgeIter h_pointer;
     bool rightLoop = true;
     bool leftLoop = true;

     // "right" part
     h_pointer = h[6]->next();
     while(h_pointer!=h[9]){
       // std::cout<<"right"<<": "<<elementAddress(h_pointer->edge())<<std::endl;
       erQueue.remove(h_pointer->edge()->record);
       h_pointer->setNeighbors(h_pointer->next(),h_pointer->twin(),v[0],h_pointer->edge(),h_pointer->face());
       // h_pointer->vertex() = vn;
       if(!h_pointer->isBoundary()){
         h_pointer = h_pointer->twin()->next();
       }
       else{
         rightLoop = false;
         break;
       }
     }
     if(!rightLoop){
       h_pointer = h[9]->next()->next()->twin();
       while (!h_pointer->isBoundary()) {
         erQueue.remove(h_pointer->edge()->record);
         h_pointer->setNeighbors(h_pointer->next(),h_pointer->twin(),v[0],h_pointer->edge(),h_pointer->face());
         // h_pointer->vertex() = vn;
         h_pointer = h_pointer->next()->next()->twin();
       }
       h_pointer->vertex() = v[0];
     }

     // "left" part
     h_pointer = h[8]->next();
     while(h_pointer!=h[7]){
       // std::cout<<"left"<<": "<<elementAddress(h_pointer->edge())<<std::endl;
       erQueue.remove(h_pointer->edge()->record);
       h_pointer->setNeighbors(h_pointer->next(),h_pointer->twin(),v[0],h_pointer->edge(),h_pointer->face());
       // h_pointer->vertex() = vn;
       if(!h_pointer->isBoundary()){
         h_pointer = h_pointer->twin()->next();
       }
       else{
         leftLoop = false;
         break;
       }
     }
     if(!leftLoop){
       h_pointer = h[7]->next()->next()->twin();
       while (!h_pointer->isBoundary()) {
         erQueue.remove(h_pointer->edge()->record);
         h_pointer->setNeighbors(h_pointer->next(),h_pointer->twin(),v[0],h_pointer->edge(),h_pointer->face());
         // h_pointer->vertex() = vn;
         h_pointer = h_pointer->next()->next()->twin();
       }
       h_pointer->vertex() = v[0];
     }

     // delete
     for(int i = 0;i<6;i++){
       deleteHalfedge(h[i]);
     }
     for(int i = 1;i<2;i++){
       deleteVertex(v[i]);
     }

     deleteEdge(e[0]);
     deleteEdge(e[2]);
     deleteEdge(e[3]);

     for(int i = 0;i<2;i++){
       deleteFace(f[i]);
     }
     return v[0];
   }

   EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
   {
      // TODO This method should flip the given edge and return an iterator to the flipped edge.
      // std::cout<<"flipEdge"<<std::endl;
      // check
      if(e0->halfedge()->face()->isBoundary()||e0->halfedge()->twin()->face()->isBoundary()){
        std::cerr << "Cannot flip a boundary." << std::endl;
        // throw EdgeEditException(11);
        return e0;
      }

      HalfedgeIter h[6];

      h[0] = e0->halfedge();
      h[1] = h[0]->next();
      h[2] = h[1]->next();
      h[3] = h[0]->twin();
      h[4] = h[3]->next();
      h[5] = h[4]->next();

      if(h[2]->twin()==h[4]->twin()->next()
      &&h[5]->twin()==h[1]->twin()->next()
      &&h[5]->twin()->next()==h[2]->twin()->next()->twin()){
        std::cerr << "Cannot flip this edge." << std::endl;
        // throw EdgeEditException(10);
        return EdgeIter();
      }

      // for(int i=0;i<6;i++){
      //   std::cout<<"h"<<i<<": "<<elementAddress(h[i])<<std::endl;
      // }

      VertexIter v[4];
      v[0] = h[0]->vertex();
      v[1] = h[1]->vertex();
      v[2] = h[2]->vertex();
      v[3] = h[5]->vertex();

      // for(int i=0;i<4;i++){
      //   std::cout<<"v"<<i<<": "<<elementAddress(v[i])<<std::endl;
      // }

      FaceIter f[2];
      f[0] = h[0]->face();
      f[1] = h[3]->face();
      // for(int i=0;i<2;i++){
      //   std::cout<<"f"<<i<<": "<<elementAddress(f[i])<<std::endl;
      // }

      h[0]->setNeighbors(h[5],h[0]->twin(),v[2],h[0]->edge(),f[0]);
      h[3]->setNeighbors(h[2],h[3]->twin(),v[3],h[3]->edge(),f[1]);

      h[5]->setNeighbors(h[1],h[5]->twin(),h[5]->vertex(),h[5]->edge(),f[0]);
      h[2]->setNeighbors(h[4],h[2]->twin(),h[2]->vertex(),h[2]->edge(),f[1]);

      h[1]->setNeighbors(h[0],h[1]->twin(),h[1]->vertex(),h[1]->edge(),f[0]);
      h[4]->setNeighbors(h[3],h[4]->twin(),h[4]->vertex(),h[4]->edge(),f[1]);

      f[0]->halfedge() = h[1];
      f[1]->halfedge() = h[4];
      e0->halfedge() = h[0];

      v[0]->halfedge() = h[4];
      v[1]->halfedge() = h[1];
      v[2]->halfedge() = h[2];
      v[3]->halfedge() = h[5];

      // std::cout<<"Done"<<std::endl;
			return e0;
   }

   void MeshResampler::upsample( HalfedgeMesh& mesh )
   // This routine should increase the number of triangles in the mesh using Loop subdivision.
   {
      // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
      // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
      // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
      // the new subdivided (fine) mesh, which has more elements to traverse.  We will then assign vertex positions in
      // the new mesh based on the values we computed for the original mesh.


      // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
      // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
      // TODO a vertex of the original mesh.
      int countv = 0;
      for(VertexIter v = mesh.verticesBegin();v!=mesh.verticesEnd();v++){
        countv++;
        v->isNew=false;
        // std::cout<<"degree: "<<n<<std::endl;
        double myWeight,uWeight;
        if(v->degree()==3){
          uWeight = 3.0/16.0;
        }
        else{
          uWeight = 3.0/(8.0 * double(v->degree()));
        }

        myWeight = 1.0 - uWeight * double(v->degree());
        // v->newPosition = v->position * myWeight;

        HalfedgeIter starth = v->halfedge();
        HalfedgeIter h = starth;

        Vector3D sumv = Vector3D(0,0,0);

        do{
          sumv += h->twin()->vertex()->position;
          // v->newPosition += h->twin()->vertex()->position * uWeight;
          h = h->next()->next()->twin();
        }while(h!=starth&&!h->isBoundary());

        v->newPosition = uWeight * sumv  + myWeight * v->position;

        // std::cout<<"v:"<<elementAddress(v)<<" newP = "<<v->newPosition<<" myWeight =  "<<myWeight<<"degree = "<<v->degree()<<std::endl;

        if(h->face()->isBoundary()){
          h = h->twin();
          v->newPosition += h->vertex()->position * uWeight;
          h = starth->twin();
          while(!h->face()->isBoundary()){
              h = h->next()->twin();
              v->newPosition += h->vertex()->position * uWeight;
          }
        }// end if isBoundary()
        // std::cout<<"newPosition: "<<v->newPosition<<std::endl;
      } // end for
      // std::cout<<"old vertices: "<<countv<<std::endl;

      // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.

      list<EdgeIter> oeList;
      int counte = 0;
      for(EdgeIter e = mesh.edgesBegin();e!=mesh.edgesEnd();e++){
        e->isNew = false;
        oeList.push_back(e);
        double uWeight = 3.0 / 8.0;
        e->newPosition = ( e->halfedge()->vertex()->position + e->halfedge()->twin()->vertex()->position ) * uWeight;
        uWeight = 1.0/8.0;
        if(!e->halfedge()->face()->isBoundary()){
          e->newPosition += e->halfedge()->next()->twin()->vertex()->position * uWeight;
        }
        if(!e->halfedge()->twin()->face()->isBoundary()){
          e->newPosition += e->halfedge()->twin()->next()->twin()->vertex()->position * uWeight;
        }
        counte++;
        // std::cout<<"newPosition: "<<e->newPosition<<std::endl;
      }
      // std::cout<<"old mesh: "<<counte<<std::endl;


      // TODO Next, we're going to split every edge in the mesh, in any order.  For future
      // TODO reference, we're also going to store some information about which subdivided
      // TODO edges come from splitting an edge in the original mesh, and which edges are new,
      // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
      // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
      // TODO just split (and the loop will never end!)
      while(!oeList.empty()){
        EdgeIter e = oeList.back();
        VertexIter nv;
        // try{
          nv = mesh.splitEdge(e);
        // }
        // catch(EdgeEditException& e){
          // e.exptInfo();
          // continue;
        // }
        // nv->isNew = true;
        nv->newPosition = e->newPosition;
        oeList.pop_back();
      }

      // TODO Now flip any new edge that connects an old and new vertex.
      counte = 0;
      for(EdgeIter e = mesh.edgesBegin();e!=mesh.edgesEnd();e++){
        if(e->isNew && (e->halfedge()->vertex()->isNew ^ e->halfedge()->twin()->vertex()->isNew)){
            mesh.flipEdge(e);
        }
        counte++;
      }
      // std::cout<<"new mesh: "<<counte<<std::endl;

      countv = 0;
      // TODO Finally, copy the new vertex positions into final Vertex::position.
      for(VertexIter v = mesh.verticesBegin();v!=mesh.verticesEnd();v++){
        countv++;
        // std::cout<<"newPosition: "<<v->newPosition<<std::endl;
        v->position = v->newPosition;
      }
      // std::cout<<"new vertices: "<<countv<<std::endl;

   }

   // Given an edge, the constructor for EdgeRecord finds the
   // optimal point associated with the edge's current quadric,
   // and assigns this edge a cost based on how much quadric
   // error is observed at this optimal point.
   EdgeRecord::EdgeRecord( EdgeIter& _edge )
   : edge( _edge )
   {
      // TODO Compute the combined quadric from the edge endpoints.
      Matrix4x4 combQ = _edge->halfedge()->vertex()->quadric + _edge->halfedge()->twin()->vertex()->quadric;

      // TODO Build the 3x3 linear system whose solution minimizes
      // the quadric error associated with these two endpoints.
      Matrix3x3 A;
      for(int i = 0;i<3;i++){
        for(int j = 0;j<3;j++){
          A(i,j) = combQ(i,j);
        }
      }

      Vector3D b = Vector3D(-combQ(0,3),-combQ(1,3),-combQ(2,3));

      // TODO Use this system to solve for the optimal position, and
      // TODO store it in EdgeRecord::optimalPoint.
      Vector3D x = A.inv() * b;
      optimalPoint = x;

      // TODO Also store the cost associated with collapsing this edge
      // TODO in EdgeRecord::Cost.
      // VertexIter v[4];
      // v[0] = _edge->halfedge()->vertex();
      // v[1] = _edge->halfedge()->twin()->vertex();
      // v[2] = _edge->halfedge()->next()->twin()->vertex();
      // v[3] = _edge->halfedge()->twin()->next()->twin()->vertex();
      // for(int i=0;i<4;i++){
      //   if(v[i]->degree()<=3){
      //       // std::cerr << "Cannot collapse this edge." << std::endl;
      //       score = numeric_limits<double>::max();
      //       throw EdgeEditException(30);
      //       // break;
      //   }
      // }

      Vector4D xh = Vector4D(x.x,x.y,x.z,1);
      score = dot(combQ*xh, xh);

      edge = _edge;


   }


   void MeshResampler::downsample( HalfedgeMesh& mesh )
   {
      // TODO Compute initial quadrics for each face by simply writing the plane
      // equation for the face in homogeneous coordinates.  These quadrics should
      // be stored in Face::quadric
      for(FaceIter f = mesh.facesBegin();f!=mesh.facesEnd();f++){
        Vector3D p = f->halfedge()->vertex()->position;
        Vector3D n = f->normal();
        double d = dot(-n,p);
        Vector4D v = Vector4D(n.x,n.y,n.z,d);
        f->quadric = outer(v,v);
        // std::cout<<f->quadric<<std::endl;
      }

      // TODO Compute an initial quadric for each vertex as the sum of the quadrics
      // associated with the incident faces, storing it in Vertex::quadric
      for(VertexIter v = mesh.verticesBegin();v!=mesh.verticesEnd();v++){
        HalfedgeIter h = v->halfedge();
        v->quadric.zero();
        do{
          v->quadric += h->face()->quadric;
          h = h->next()->next()->twin();
        }while(h!=v->halfedge()&&!h->isBoundary());

        if(h->isBoundary()){
          h = v->halfedge()->twin();
          do{
            v->quadric += h->face()->quadric;
            h = h->next()->twin();
          }while(h!=v->halfedge()->twin()&&!h->isBoundary());
        }
        // std::cout<<v->quadric<<std::endl;
      }


      // TODO Build a priority queue of edges according to their quadric error cost,
      // TODO i.e., by building an EdgeRecord for each edge and sticking it in the queue.
      // MutablePriorityQueue<EdgeRecord> erQueue;
      erQueue.clear();
      for(EdgeIter e = mesh.edgesBegin();e!=mesh.edgesEnd();e++){
        // erQueue.remove(e->record);
        EdgeRecord er;
        // try{
          er = EdgeRecord(e);
        // }
        // catch (EdgeEditException& exception){
        //   exception.exptInfo();
        //   std::cerr << "Cannot downsample this mesh." << std::endl;
        //   return;
        // }
        e->record = er;
        erQueue.insert(er);
      }

      // TODO Until we reach the target edge budget, collapse the best edge.  Remember
      // TODO to remove from the queue any edge that touches the collapsing edge BEFORE
      // TODO it gets collapsed, and add back into the queue any edge touching the collapsed
      // TODO vertex AFTER it's been collapsed.  Also remember to assign a quadric to the
      // TODO collapsed vertex, and to pop the collapsed edge off the top of the queue.
      int _nFaces = mesh.nFaces();
      int _nVertices = mesh.nVertices();

      int reduceFaces = _nFaces/10+1;
      if(_nFaces<=4){
        std::cerr<<"Cannot downsample anymore."<<std::endl;
        return;
      }

      std::cout<<"nFaces: "<<_nFaces<<endl;
      std::cout<<"nVertices: "<<_nVertices<<endl;
      while(reduceFaces>0){
        reduceFaces--;
        // Get the cheapest edge from the queue.
        EdgeRecord er= erQueue.top();
        EdgeIter e = er.edge;

        // Remove the cheapest edge from the queue by calling pop().
        erQueue.pop();

        // Compute the new quadric by summing the quadrics at its two endpoints.
        Matrix4x4 newQ = e->halfedge()->vertex()->quadric + e->halfedge()->twin()->vertex()->quadric;

        // Remove any edge touching either of its endpoints from the queue. (Has been done in collapse)
        // Collapse the edge.
        VertexIter vn = mesh.collapseEdge(e);

        // Set the quadric of the new vertex to the quadric computed in Step 2.
        vn->quadric = newQ;

        // Insert any edge touching the new vertex into the queue, creating new edge records for each of them.
        HalfedgeIter h = vn->halfedge();
        do{
          EdgeRecord inse;
          inse = EdgeRecord(h->edge());
          erQueue.insert(inse);
          h = h->next()->next()->twin();
        }while(h!=vn->halfedge()&&!h->face()->isBoundary());
      }
}

   void Vertex::computeCentroid( void )
   {
      // TODO Compute the average position of all neighbors of this vertex, and
      // TODO store it in Vertex::centroid.  This value will be used for resampling.
   }

   Vector3D Vertex::normal( void ) const
   // TODO Returns an approximate unit normal at this vertex, computed by
   // TODO taking the area-weighted average of the normals of neighboring
   // TODO triangles, then normalizing.
   {
      // TODO Compute and return the area-weighted unit normal.
			return Vector3D();
	 }

   void MeshResampler::resample( HalfedgeMesh& mesh )
   {
      // TODO Compute the mean edge length.


      // TODO Repeat the four main steps for 5 or 6 iterations


      // TODO Split edges much longer than the target length (being careful about how the loop is written!)


      // TODO Collapse edges much shorter than the target length.  Here we need to be EXTRA careful about
      // TODO advancing the loop, because many edges may have been destroyed by a collapse (which ones?)

      //
      // TODO Now flip each edge if it improves vertex degree


      // TODO Finally, apply some tangential smoothing to the vertex positions
   }




}
