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
      const int* getSize() const;
      double& operator()(int i, int j);
      const double& operator()(int i, int j) const;
      ~Matrix();
      void resize(int newRows, int newCols);
      Matrix(const Matrix& other);
      Matrix& operator=(const Matrix& other);
};