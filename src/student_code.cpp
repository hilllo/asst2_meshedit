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
   VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
   {
      // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
      // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
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
      e[0] = e0;
      for(int i=1;i<4;i++){
        e[i] = newEdge();
      }
      // assign
      e[1]->halfedge() = h[3];
      e[2]->halfedge() = h[6];
      e[3]->halfedge() = h[11];

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

			return v[4];
	 }

   VertexIter HalfedgeMesh::collapseEdge( EdgeIter e )
   {
      // TODO This method should collapse the given edge and return an iterator to the new vertex created by the collapse.

			return VertexIter();
   }

   EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
   {
      // TODO This method should flip the given edge and return an iterator to the flipped edge.
      // std::cout<<"flipEdge"<<std::endl;

      HalfedgeIter h[6];

      h[0] = e0->halfedge();
      h[1] = h[0]->next();
      h[2] = h[1]->next();
      h[3] = h[0]->twin();
      h[4] = h[3]->next();
      h[5] = h[4]->next();

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


      // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.


      // TODO Next, we're going to split every edge in the mesh, in any order.  For future
      // TODO reference, we're also going to store some information about which subdivided
      // TODO edges come from splitting an edge in the original mesh, and which edges are new,
      // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
      // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
      // TODO just split (and the loop will never end!)


      // TODO Now flip any new edge that connects an old and new vertex.


      // TODO Finally, copy the new vertex positions into final Vertex::position.
   }

   // Given an edge, the constructor for EdgeRecord finds the
   // optimal point associated with the edge's current quadric,
   // and assigns this edge a cost based on how much quadric
   // error is observed at this optimal point.
   EdgeRecord::EdgeRecord( EdgeIter& _edge )
   : edge( _edge )
   {
      // TODO Compute the combined quadric from the edge endpoints.


      // TODO Build the 3x3 linear system whose solution minimizes
      // the quadric error associated with these two endpoints.


      // TODO Use this system to solve for the optimal position, and
      // TODO store it in EdgeRecord::optimalPoint.


      // TODO Also store the cost associated with collapsing this edge
      // TODO in EdgeRecord::Cost.

   }

   void MeshResampler::downsample( HalfedgeMesh& mesh )
   {
      // TODO Compute initial quadrics for each face by simply writing the plane
      // equation for the face in homogeneous coordinates.  These quadrics should
      // be stored in Face::quadric


      // TODO Compute an initial quadric for each vertex as the sum of the quadrics
      // associated with the incident faces, storing it in Vertex::quadric


      // TODO Build a priority queue of edges according to their quadric error cost,
      // TODO i.e., by building an EdgeRecord for each edge and sticking it in the queue.


      // TODO Until we reach the target edge budget, collapse the best edge.  Remember
      // TODO to remove from the queue any edge that touches the collapsing edge BEFORE
      // TODO it gets collapsed, and add back into the queue any edge touching the collapsed
      // TODO vertex AFTER it's been collapsed.  Also remember to assign a quadric to the
      // TODO collapsed vertex, and to pop the collapsed edge off the top of the queue.
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