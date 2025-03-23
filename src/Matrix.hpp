class Matrix
{
    private:
      int sizeMat[2];
      double* matPointer;
    public:
      Matrix(int size[2]);
      double* getMatrix();
      int* getSize();
      double& operator()(int i, int j);
    //   ~Matrix();
};