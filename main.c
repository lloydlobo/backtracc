#include <stdio.h>
#include <stdlib.h>

#define N          9
#define MOVES_4DIR 4

enum Cell_Kind {
    CELL_OPEN     = 0,
    CELL_OBSTACLE = 1,
    CELL_VISITED  = 2,
};

struct Coordinates {
    int x;
    int y;
};

const struct Coordinates Start_Coord = {2, 2};
const struct Coordinates End_Coord   = {(N - 3), (N - 3)};

const struct Coordinates Offsets_Dx_Dy[MOVES_4DIR] = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1}};

const char Directions[MOVES_4DIR] = {'U', 'D', 'L', 'R'};

void print_horizontal_line(int repeat, const char pattern) {
    printf(" ");
    for (int i = 0; i < repeat; i++) printf("%c", pattern);
    printf(" ");
    printf("\n");
};

int is_start(int x, int y) { return x == Start_Coord.x && y == Start_Coord.y; }
int is_end(int x, int y) { return x == End_Coord.x && y == End_Coord.y; }

void print_maze(const int maze[N][N]) {
    int gap = 3;  // '%2c ' (2+1) cell_char space.

    print_horizontal_line((N * gap), '=');

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            char cell_char;

            switch (maze[i][j]) {
            case CELL_OPEN:
                if (is_start(i, j)) cell_char = 'A';     // start.
                else if (is_end(i, j)) cell_char = 'B';  // end.
                else cell_char = '-';                    // unvisited.
                break;
            case CELL_OBSTACLE: cell_char = 'x'; break;  // obstacle.
            case CELL_VISITED:
                if (is_end(i, j)) cell_char = 'B';          // end.
                else if (!is_start(i, j)) cell_char = '.';  // path point.
                else if (is_start(i, j)) cell_char = 'A';   // start.
                else cell_char = '?';                       // logic error.
                break;
            default:
                fprintf(stderr, "error: invalid maze cell: %i\n", maze[i][j]);
                exit(1);
            }

            if (j == 0) printf("|%2c ", cell_char);
            else if (j == (N - 1)) printf("%2c |", cell_char);
            else printf("%2c ", cell_char);
        }
        printf("\n");
    }

    print_horizontal_line((N * gap), '=');
}

int is_valid_move(const int board[N][N], int x, int y) {
    int is_in_bounds = ((x >= 0) && (x < N) && (y >= 0) && (y < N));
    return ((board[x][y] == CELL_OPEN) && is_in_bounds);
}

int backtrack(int board[N][N], int x, int y, char path[], int path_len) {
    if (is_end(x, y)) {                                  // Check base case.
        board[End_Coord.x][End_Coord.y] = CELL_VISITED;  // Mark as visited.
        path[path_len] = '\0';  // nul terminate as we got final path.
        return 1;               // End point reached.
    }

    for (int i = 0; i < MOVES_4DIR; i += 1) {
        int nx = (x + Offsets_Dx_Dy[i].x), ny = (y + Offsets_Dx_Dy[i].y);

        if (is_valid_move(board, nx, ny)) {
            board[nx][ny]  = CELL_VISITED;   // Mark move as visited.
            path[path_len] = Directions[i];  // Record path direction.
            if (backtrack(board, nx, ny, path, (path_len + 1)))
                return 1;               // Valid path cell visited.
            board[nx][ny] = CELL_OPEN;  // Mark move as unvisited(backtrack).
        }
    }

    return 0;  // No path found.
}

int main(void) {
    int maze_board[N][N] = {
        // clang-format off
    {0, 1, 0, 0, 0, 0, 0, 1, 0},
    {0, 1, 0, 1, 1, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 0, 1, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 0, 0, 0, 1, 0},
    {0, 1, 0, 1, 1, 1, 0, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 0, 1, 1, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, 0},
        // clang-format on
    };

    char path[N * N];  // To store the path.

    if (backtrack(maze_board, Start_Coord.x, Start_Coord.y, path, 0))
        printf("Path found: %s\n", path);
    else printf("No path found\n");

    print_maze(maze_board);

    return 0;
}
