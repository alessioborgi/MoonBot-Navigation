#pragma once

#include <Eigen/Core>
#include <iostream>
#include <vector>

/**
 * @brief Grid_ represents a 2D grid of arbitrary cell type.
 * It wraps around a 1D vector of cells and provides access to them
 * through 2D indices.
 * The grid is row-major, meaning that the cells are stored in a
 * 1D vector in row-major order.
 *
 * @tparam CellType_ The type of the cells in the grid.
 */
template <typename CellType_>
struct Grid_ {
  using CellType = CellType_;
  using ContainerType = std::vector<CellType>;

  unsigned int _rows;  // Number of rows in the grid.
  unsigned int _cols;  // Number of columns in the grid.

  ContainerType _cells;  // 1D vector storing the grid cells in row-major order.

  /**
   * @brief Constructs a grid with the given number of rows and columns.
   *
   * @param rows Number of rows in the grid.
   * @param cols Number of columns in the grid.
   */
  Grid_(unsigned int rows = 0, unsigned int cols = 0)
      : _rows(rows), _cols(cols), _cells(rows * cols) {}

  inline unsigned int rows() const { return _rows; }

  inline unsigned int cols() const { return _cols; }

  inline unsigned int size() const { return _rows * _cols; }

  /**
   * @brief Accesses a cell at the given row and column.
   *
   * @param row Row index.
   * @param col Column index.
   * @return Reference to the cell.
   */
  inline CellType& at(unsigned int row, unsigned int col) {
    return _cells[row * _cols + col];
  }

  /**
   * @brief Accesses a cell at the given row and column (const version).
   *
   * @param row Row index.
   * @param col Column index.
   * @return Const reference to the cell.
   */
  inline const CellType& at(unsigned int row, unsigned int col) const {
    return _cells[row * _cols + col];
  }

  /**
   * @brief Accesses a cell at the given 2D index.
   *
   * @param idx 2D index (Eigen::Vector2i).
   * @return Const reference to the cell.
   */
  inline const CellType& at(Eigen::Vector2i idx) const {
    return at(idx.y(), idx.x());
  }

  /**
   * @brief Accesses a cell at the given 2D index.
   *
   * @param idx 2D index (Eigen::Vector2i).
   * @return Reference to the cell.
   */
  inline CellType& at(Eigen::Vector2i idx) { return at(idx.y(), idx.x()); }

  /**
   * @brief Checks if a cell is inside the grid.
   *
   * @param row Row index.
   * @param col Column index.
   * @return True if the cell is inside the grid, false otherwise.
   */
  inline bool inside(unsigned int row, unsigned int col) const {
    return row < _rows and col < _cols;
  }

  /**
   * @brief Checks if a cell is inside the grid.
   *
   * @param idx 2D index (Eigen::Vector2i).
   * @return True if the cell is inside the grid, false otherwise.
   */
  inline bool inside(Eigen::Vector2i idx) const {
    return inside(idx.y(), idx.x());
  }

  /**
   * @brief Resizes the grid to the given number of rows and columns.
   *
   * @param rows New number of rows.
   * @param cols New number of columns.
   */
  void resize(unsigned int rows, unsigned int cols) {
    _rows = rows;
    _cols = cols;
    _cells.resize(rows * cols);
  }

  /**
   * @brief Computes the <row, col> indices given a pointer to a cell.
   *
   * @param ptr Pointer to a cell.
   * @return std::pair<unsigned int, unsigned int> Pair of row and column
   * indices.
   */
  inline std::pair<unsigned int, unsigned int> getCoordinatesFromPointer(
      const CellType* ptr) const {
    unsigned int idx = ptr - &_cells[0];
    return std::make_pair(idx / _cols, idx % _cols);
  }
};