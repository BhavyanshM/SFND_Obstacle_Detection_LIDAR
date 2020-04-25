/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

  	void actualInsert(Node** parent, std::vector<float> newPoint, int id, int depth){
    	if (*parent == NULL){
        	*parent = new Node(newPoint,id);
        }else{
       		if (depth % 2 == 0){
              if(newPoint[0] < (*parent)->point[0]){
                actualInsert(&((*parent)->left), newPoint, id, depth+1);
              }else{
                actualInsert(&((*parent)->right), newPoint, id, depth+1);
              }
            }else{
              if(newPoint[1] < (*parent)->point[1]){
                actualInsert(&((*parent)->left), newPoint, id, depth+1);
              }else{
                actualInsert(&((*parent)->right), newPoint, id, depth+1);
              }
            }
        }
      	

    }
  
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		actualInsert(&root,point,id,0);
    }

  	void actualSearch(Node* node, int depth, std::vector<float> target, float distanceTol, std::vector<int>& ids){
      if(node!=NULL){
      		
        	if(node->point[0]>=(target[0]-distanceTol) && 
              node->point[0]<=(target[0]+distanceTol) &&
              node->point[1]>=(target[1]-distanceTol) &&
              node->point[1]>=(target[1]-distanceTol)){
            
            	float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) +
                                     (node->point[1]-target[1])*(node->point[1]-target[1]));
              	
              	if(distance <= distanceTol){
                	ids.push_back(node->id);
                }
            }
        
        	if (depth % 2 == 0){
              if(target[0]-distanceTol < node->point[0]){
                actualSearch(node->left,depth+1,target,distanceTol,ids);
              }
              if(target[0]+distanceTol > node->point[0]){
                actualSearch(node->right,depth+1,target,distanceTol,ids);
              }
            }else{
              if(target[1]-distanceTol < node->point[1]){
                actualSearch(node->left,depth+1,target,distanceTol,ids);
              }
              if(target[1]+distanceTol > node->point[1]){
                actualSearch(node->right,depth+1,target,distanceTol,ids);
              }
            }
      }
    		
    }
  
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
      
      	actualSearch(root, 0, target, distanceTol, ids);
      
		return ids;
	}
	

};




 