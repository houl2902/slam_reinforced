#pragma once
class Matrix
{
    private:
      int sizeMat[2];
      float* matPointer;
    public:
      Matrix(int size[2]);
      Matrix(int x, int y);
      float* getMatrix();
      int* getSize();
      const int* getSize() const;
      float& operator()(int i, int j);
      const float& operator()(int i, int j) const;
      ~Matrix();
      void resize(int newRows, int newCols);
      Matrix(const Matrix& other);
      Matrix& operator=(const Matrix& other);
};