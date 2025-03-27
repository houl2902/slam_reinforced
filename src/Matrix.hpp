class Matrix
{
    private:
      int sizeMat[2];
      double* matPointer;
    public:
      Matrix(int size[2]);
      Matrix(int x, int y);
      double* getMatrix();
      int* getSize();
      double& operator()(int i, int j);
    //   ~Matrix();
};