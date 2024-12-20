#include <map>
#include <unordered_set>
#include "utils.hpp"

#define TESTING 0
#if TESTING
#define INFILE "testInput.txt"
#else
#define INFILE "input.txt"
#endif

struct Coord
{
	int x;
	int y;
};

struct CoordHasher
{
	size_t operator()(Coord const& coord) const
	{
		static_assert(sizeof(Coord) == 2 * sizeof(int));
		return *reinterpret_cast<size_t const*>(&coord);
	}
};

struct CoordPairHasher
{
	size_t operator()(std::pair<Coord, Coord> const& coordPair) const
	{
		size_t out = 0;
		out |= coordPair.first.x;
		out |= static_cast<size_t>(coordPair.second.x) << 16;
		out |= static_cast<size_t>(coordPair.first.y) << 32;
		out |= static_cast<size_t>(coordPair.second.y) << 48;
		return out;
	}
};

bool operator==(Coord const& lhs, Coord const& rhs)
{
	return lhs.x == rhs.x && lhs.y == rhs.y;
}

TwoDVector<int> Dijkstras(TwoDVector<char> const& input)
{
	Coord raceStart{ 0 };
	Coord raceEnd{ 0 };

	TwoDVector<int> costs(input.XDim());
	costs.resize(input.XDim() * input.YDim());

	std::fill(costs.begin(), costs.end(), INT32_MAX);

	for (int y = 0; y < input.YDim(); ++y)
	{
		for (int x = 0; x < input.XDim(); ++x)
		{
			if (input[y][x] == 'S')
			{
				raceStart = Coord{ .x = x, .y = y };
			}
			if (input[y][x] == 'E')
			{
				raceEnd = Coord{ .x = x, .y = y };
			}
		}
	}

	std::map<size_t, std::vector<Coord>> candidates = { {0, {raceEnd}} };
	costs[raceEnd.y][raceEnd.x] = 0;

	while (!candidates.empty())
	{
		auto [cost, nextCoords] = *candidates.begin();
		candidates.erase(cost);

		for (auto const& coord : nextCoords)
		{
			for (auto const& [dirInt, direction] : DirectionIterator())
			{
				Coord nextCoord = { 
					.x = coord.x + DirectionToX(direction), 
					.y = coord.y + DirectionToY(direction) 
				};

				if (!input.IsInBounds(nextCoord.x, nextCoord.y)) continue;
				if (input[nextCoord.y][nextCoord.x] == '#') continue;

				int nextCost = costs[coord.y][coord.x] + 1;
				int& currentNextCost = costs[nextCoord.y][nextCoord.x];

				if (nextCost >= currentNextCost) continue;

				if (currentNextCost != INT32_MAX)
				{
					EasyErase(candidates[currentNextCost], nextCoord);
					if (candidates[currentNextCost].empty())
					{
						candidates.erase(currentNextCost);
					}
				}

				currentNextCost = nextCost;
				candidates[nextCost].push_back(nextCoord);
			}
		}
	}

	return costs;
}

TwoDVector<int> SimplePathFind(TwoDVector<char> const& input)
{
	Coord raceStart{ 0 };
	Coord raceEnd{ 0 };

	TwoDVector<int> costs(input.XDim());
	costs.resize(input.XDim() * input.YDim());

	std::fill(costs.begin(), costs.end(), INT32_MAX);

	for (int y = 0; y < input.YDim(); ++y)
	{
		for (int x = 0; x < input.XDim(); ++x)
		{
			if (input[y][x] == 'S')
			{
				raceStart = Coord{ .x = x, .y = y };
			}
			if (input[y][x] == 'E')
			{
				raceEnd = Coord{ .x = x, .y = y };
			}
		}
	}

	Coord coord = raceEnd;
	int cost = 0;

	while (coord != raceStart)
	{
		costs[coord.y][coord.x] = cost;
		++cost;

		for (auto [_, direction] : DirectionIterator())
		{
			Coord nextCoord = {
					.x = coord.x + DirectionToX(direction),
					.y = coord.y + DirectionToY(direction)
			};

			if (!costs.IsInBounds(nextCoord.x, nextCoord.y)) continue;
			if (input[nextCoord.y][nextCoord.x] == '#') continue;
			if (costs[nextCoord.y][nextCoord.x] != INT32_MAX) continue;

			coord = nextCoord;
			break;
		}
	}

	costs[coord.y][coord.x] = cost;
	return costs;
}

std::map<int, int> CountCheats(TwoDVector<int> const& costGrid)
{
	std::unordered_set<std::pair<Coord, Coord>, CoordPairHasher> cheatsSeen;
	std::map<int, int> cheats;
	for (int y = 0; y < costGrid.YDim(); ++y)
	{
		for (int x = 0; x < costGrid.XDim(); ++x)
		{
			int currentCost = costGrid[y][x];
			if (currentCost == INT32_MAX) continue;
			for(int yStep = -20; yStep <= 20; ++yStep)
			{
				for (int xStep = -20; xStep <= 20; ++xStep)
				{
					int cheatCost = abs(xStep) + abs(yStep);
					if (cheatCost > 20) continue;
					int nextY = y + yStep;
					int nextX = x + xStep;

					if (!costGrid.IsInBounds(nextX, nextY)) continue;
					int nextCost = costGrid[nextY][nextX];
					if (nextCost == INT32_MAX) continue;
					nextCost += cheatCost;
					if (nextCost >= currentCost) continue;

					auto cheatId = std::make_pair(Coord{ .x = x, .y = y }, Coord{ .x = nextX, .y = nextY });
					if (cheatsSeen.count(cheatId)) continue;
					cheatsSeen.insert(cheatId);

					++cheats[currentCost - nextCost];
				}
			}
		}
	}
	return cheats;
}

int ScoreCheats(TwoDVector<int> const& costGrid, int cheatPeriod)
{
	int score = 0;
	for (int y = 0; y < costGrid.YDim(); ++y)
	{
		for (int x = 0; x < costGrid.XDim(); ++x)
		{
			int currentCost = costGrid[y][x];
			if (currentCost == INT32_MAX) continue;
			if (currentCost < 100) continue;

			int yMin = std::max(0, y - cheatPeriod);
			int yMax = std::min<int>(costGrid.YDim() - 1, y + cheatPeriod);

			for (int nextY = yMin; nextY <= yMax; ++nextY)
			{
				//if (!costGrid.IsInBounds(0, nextY)) Unreachable();

				int remainingSteps = cheatPeriod - abs(y - nextY);
				int xMin = std::max(0, x - remainingSteps);
				int xMax = std::min<int>(costGrid.XDim() - 1, x + remainingSteps);

				for (int nextX = xMin; nextX <= xMax; ++nextX)
				{
					//if (!costGrid.IsInBounds(nextX, nextY)) Unreachable();


					int nextCost = costGrid[nextY][nextX];
					if (nextCost == INT32_MAX) continue;

					int cheatCost = abs(nextX - x) + abs(nextY - y);
					nextCost += cheatCost;

					if (nextCost + 100 > currentCost) continue;

					++score;
				}
			}
		}
	}
	return score;
}

void PrintGrid(TwoDVector<char> const& input, TwoDVector<int> const& costGrid)
{
	for (int y = 0; y < costGrid.YDim(); ++y)
	{
		for (int x = 0; x < costGrid.XDim(); ++x)
		{
			if (costGrid[y][x] == INT32_MAX)
			{
				if (input[y][x] != '#') Unreachable();
				printf(" # ");
				continue;
			}
			printf("%2d ", costGrid[y][x]);
		}
		printf("\n");
	}
}

int Score(std::map<int, int> const& cheats)
{
	int total = 0;
	for (auto itr = cheats.lower_bound(100); itr != cheats.end(); ++itr)
	{
		auto const& [score, count] = *itr;
		if (score < 100) continue;
		total += count;
	}
	return total;
}


#include <chrono>

int main()
{
	auto start = std::chrono::high_resolution_clock::now();
	auto inputLines = GetInputAsString(INFILE);
	auto inputGrid = GetInputGrid<char>(INFILE);

	//auto costs = Dijkstras(inputGrid);
	auto costs = SimplePathFind(inputGrid);
	auto p1 = ScoreCheats(costs, 2);
	auto p2 = ScoreCheats(costs, 20);
	auto end = std::chrono::high_resolution_clock::now();

	printf("p1: %d\n", p1);
	printf("p2: %d\n", p2);
	printf("%lld ms\n", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
}