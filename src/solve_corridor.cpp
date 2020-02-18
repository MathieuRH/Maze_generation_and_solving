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

    Position(int _x, int _y, int distance) : Point(_x, _y), dist(distance) {}

    int distToParent()
    {
        return dist;
    }

    void initStartGoal(Position start, Position goal)
    {
        // On initialise les points de départ et d'arrivée
        x_start = start.x;
        y_start = start.y;
        x_goal = goal.x;
        y_goal = goal.y;
    }

    bool is_corridor(int _x, int _y, int &i, int &j)
    {
        // Si on vient d'un des côtés et qu'une seule autre case est libre
        if(i && maze.isFree(_x+i, _y) +
                maze.isFree(_x, _y+1) +
                maze.isFree(_x, _y-1) == 1)
        {
            // En fonction de la case libre, on choisit la case du dessus ou du dessous
            if(maze.isFree(_x, _y-1))
            {
                i = 0;
                j = -1;
            }
            else if(maze.isFree(_x, _y+1))
            {
                i = 0;
                j = 1;
            }
            return true;
        }
        // Si on vient d'au dessus ou d'en dessous et qu'une seule autre case est libre
        else if(j && maze.isFree(_x, _y+j) +
                maze.isFree(_x+1, _y) +
                maze.isFree(_x-1, _y) == 1)
        {
            // En fonction de la case libre, on choisit la case de droite ou de gauche
            if(maze.isFree(_x-1, _y))
            {
                i = -1;
                j = 0;
            }
            else if(maze.isFree(_x+1, _y))
            {
                i = 1;
                j = 0;
            }
            return true;
        }
        return false; // dans aucun de ces deux cas, ce n'est pas un corridor
    }

    void show(bool closed, const Point &parent)
    {
        const int b = closed?255:0, r = closed?0:255;
        std::vector<Pair> case_suiv;
        case_suiv={{-1,0},{1,0},{0,-1},{0,1}};

        for (auto direction : case_suiv){ // On parcourt les quatres directions
            int i = direction.first;
            int j = direction.second;

            int x_parent(parent.x + i), y_parent(parent.y + j);
            if (maze.isFree(x_parent, y_parent)) // si la case suivante dans la direction étudiée est libre
            {
                // tant qu'on suit un corridor
                while(is_corridor(x_parent, y_parent, i, j))
                {
                    maze.write(x_parent, y_parent, r, 0, b, false);// affiche la position
                    x_parent += i;
                    y_parent += j;// incrémentation des cases
                }
            }
        }
        // on affiche le chemin en cours
        maze.write(x,y,r,0,b,true);
    }

    void print(const Point &parent)
    {
        std::vector<Pair> case_suiv;
        case_suiv={{-1,0},{1,0},{0,-1},{0,1}};

        for (auto direction : case_suiv){ // On parcourt les quatres directions
            int i = direction.first;
            int j = direction.second;

            vector<std::pair<int, int>> chemin; // chemin est le chemin reliant le parent à son enfant

            int x_parent(parent.x + i), y_parent(parent.y + j);
            if (maze.isFree(x_parent, y_parent)) // si la case suivante dans la direction étudiée est libre
            {
                chemin.push_back({x_parent, y_parent}); // on ajoute au chemin la case précédente
                // tant qu'on est dans un corridor
                while(is_corridor(x_parent, y_parent, i, j) && (x_parent != x_start || y_parent != y_start))
                {
                    x_parent += i;
                    y_parent += j; // icrémentation de la case
                    chemin.push_back({x_parent, y_parent}); // on ajoute au chemin la case précédente
                }
                // Si on finit par atteindre l'enfant on trace le chemin
                if (x_parent == x && y_parent == y)
                {
                    for (auto p : chemin)
                    {
                        maze.passThrough(p.first, p.second); // écrit le chemin final
                    }
                    // On sort de la boucle pour éviter de tracer les autres directions inutiles
                    break;
                }
            }
        }
        maze.passThrough(x, y);
    }

    std::vector<PositionPtr> children()
    {

        std::vector<PositionPtr> generated;
        std::vector<Pair> case_suiv;
        case_suiv={{-1,0},{1,0},{0,-1},{0,1}};

        for (auto direction : case_suiv){ // On parcourt les quatres directions
            int i = direction.first;
            int j = direction.second;

            int x_prochain(x+i), y_prochain(y+j);
            int k = 0;
            // Si la direction est libre
            if (maze.isFree(x_prochain, y_prochain))
            {
                k = 1;
                // tant qu'on est dans un corridor et qu'on n'a pas atteint l'arrivée
                while (is_corridor(x_prochain, y_prochain, i, j) && (x_prochain != x_goal || y_prochain != y_goal))
                {
                    k++;
                    x_prochain += i;
                    y_prochain += j; //incrémentation de la case
                }
                generated.push_back(std::make_unique<Position>(x_prochain, y_prochain, k));
            }
        }
        return generated;
    }

protected:
    int dist;
    static int x_goal, y_goal, x_start, y_start; // permet de garder en mémoire les positions du début et de la fin du maze
};

int Position::x_goal, Position::y_goal, Position::x_start, Position::y_start;

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

    start.initStartGoal(start, goal);  // permet de garder en mémoire les positions du début et de la fin du maze

    // call A* algorithm
    ecn::Astar(start, goal);

    // save final image
    Position::maze.saveSolution("cell");
    cv::waitKey(0);

}
