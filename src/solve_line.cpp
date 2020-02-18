#include <a_star.h>
#include <maze.h>

using namespace std;
using namespace ecn;

// a node is a x-y position, we move from 1 each time
class Position : public Point
{
    typedef std::unique_ptr<Position> PositionPtr;

public:
    // constructor from coordinates
    Position(int _x, int _y) : Point(_x, _y) {}

    // constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}
    //constructor avec distance
    Position(int x, int y, int distance): Point(x, y), dist(distance){}


    bool isCorridor(int i, int j){
        if (!maze.isFree(i,j)){
            return false;
        }
        else {

            return maze.isFree(i+1, j) + maze.isFree(i-1, j)
                    + maze.isFree(i, j+1) + maze.isFree(i, j-1) == 2;
        }
    }
    int distToParent()
    {

        return dist;
    }

    std::vector<PositionPtr> children()
    {
        // this method should return  all positions reachable from this one
        std::vector<PositionPtr> generated;
        std::vector<Pair> case_suiv;
        case_suiv={{-1,0},{1,0},{0,-1},{0,1}}; // on interdit la postion (0,0) car elle correspond au parent

        for (auto direction : case_suiv){
            int i = direction.first;
            int j = direction.second;

            int k=1;
            while(isCorridor(x+i*k,y+j*k)){
                k++;

            }
            if (!maze.isFree(x+i*k,y+j*k)){
                k--;

            }

            if(k>0){

                    generated.push_back(make_unique<Position>(x+i*k,y+j*k,k));


                }

        }
        return generated;

    }
protected:
    int dist;
};



int main( int argc, char **argv )
{
    // load file
    std::string filename = "maze.png";
    if(argc == 2)
        filename = std::string(argv[1]);

    // let Point know about this maze
    Position::maze.load(filename);

    // initial and goal positions as Position's
    Position start = Position::maze.start(),
             goal = Position::maze.end();

    // call A* algorithm
    ecn::Astar(start, goal);

    // save final image
    Position::maze.saveSolution("line");
    cv::waitKey(0);

}
