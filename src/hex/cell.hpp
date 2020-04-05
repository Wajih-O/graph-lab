/**
 * @author Wajih Ouertani
 * @email wajih.ouertani@gmail.com
 * @create date 2020-04-05 02:51:57
 * @modify date 2020-04-05 03:42:16
 */

#ifndef MYLIB_HEX_CELL
#define MYLIB_HEX_CELL

#include <ostream>

namespace mylib {

template <class T> class HexCell {
public:
  HexCell(std::size_t i_pos, std::size_t j_pos, T cell_content)
      : i_pos(i_pos), j_pos(j_pos), content(cell_content) {}
  std::pair<std::size_t, std::size_t> get_position() {
    return std::make_pair(i_pos, j_pos);
  }
  bool operator==(const HexCell<T> &other) {
    return get_position() == other.get_position();
  }

protected:
  T content; // cell content
  std::size_t i_pos, j_pos; // position in the board
};

enum class HEX_DIRECTION { EAST, WEST, NORTH, SOUTH, NORTH_EAST, SOUTH_WEST };
enum class GAME_EDGE { EAST, WEST, NORTH, SOUTH };

// TODO: implement CELL (as abstract class) /EDGE_CELL/ GRID_CELL

class BaseCell {
public:
  virtual bool operator==(const BaseCell &bc) = 0;
};

/**
 * A special cell type where every cell edge Hex cell on the board should be
 * attached to we need one for each of the edges namely: WEST, NORTH, EAST,
 * SOUTH they are special Cells/nodes to test if a player has connected the
 * north to south, or the west to
 */
class FunnelCell : public BaseCell {

public:
  FunnelCell() {}
  virtual bool operator==(const BaseCell &other) { return false; }
   friend std::ostream &operator<<(std::ostream &out, const FunnelCell &cell) {
    out <<"(Funnel)"  << std::endl;
    return out ;
  }

};

template <class T> class Cell : public BaseCell {
public:
  Cell(std::pair<std::size_t, std::size_t> coordinates)
      : coordinates(coordinates) {}
  Cell(size_t first, size_t second) {
    coordinates = std::make_pair(first, second);
  }

  std::pair<std::size_t, std::size_t> get_coordinate() const {
    return coordinates;
  }

  size_t first() const { return this->coordinates.first; }
  size_t second() const { return this->coordinates.second; }
  size_t row() { return this->first(); }
  size_t col() { return this->second(); }

  virtual bool operator==(const Cell &other) {
    return ((this->first() == other.first()) &&
            (this->second() == other.second()));
  }
  virtual bool operator==(const BaseCell &other) { return false; }

  friend std::ostream &operator<<(std::ostream &out, const Cell<T> &cell) {
    out <<"(" << cell.first() << "," << cell.second() << ")"  << std::endl;
    return out;
  }

  friend std::ostream &operator<<(std::ostream &out, const std::shared_ptr<Cell<T>> &cell) {
    out <<"(" << cell->first() << "," << cell->second() << ")"  << std::endl;
    return out;
  }

private:
  T value;
  std::pair<size_t, size_t> coordinates;
};

}
#endif
