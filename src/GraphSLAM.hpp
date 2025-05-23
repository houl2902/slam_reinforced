#include "MatrixFunctions.hpp"

class GraphSLAM
{
    void addPose(Matrix& current_pos);
    Matrix* detectLoop(Matrix& current_pos,Matrix* history_poses);

}