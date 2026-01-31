#include "k2_tasks.h"
#include "debug.h"
#include "fmt.h"
#include "kit_algorithm.h"
#include "name_server.h"
#include "ring_buffer.h"
#include "syscalls.h"
#include "task_queue.h"
#include "uart.h"
#include <cstdint>

namespace {

// Game config.
constexpr const char* RPS_SERVER_NAME = "rps_server";
constexpr size_t MAX_QUEUE_SIZE = 2;
constexpr size_t MAX_LOBBY = 32;

// Magic enums.
enum class RPSMessageType : uint8_t {
  SIGNUP,
  LOBBY_ASSIGNED,
  NO_AVAILABLE_LOBBY,
  PLAY,
  MATCH_RESULT,
  QUIT,
};
enum class PlayType : uint8_t { EMPTY, ROCK, PAPER, SCISSORS };

// Request to sign up to the RPS game.
struct SignUpMessage {};

// Found a match response.
struct LobbyAssignedMessage {
  int lobbyId;
};

// No available lobby.
struct NoAvailableLobbyMessage {};

// Request to play the RPS move in a given lobby.
struct PlayMessage {
  int lobbyId;
  PlayType play;
};

// Match result response.
struct MatchResultMessage {
  PlayType myPlay;
  PlayType opponentPlay;
};

// Request to quit the queue or lobby.
struct QuitMessage {
  int lobbyId;
};

// Composite messages.
struct RPSMessage {
  RPSMessageType type;
  union {
    SignUpMessage signUpMessage;
    LobbyAssignedMessage lobbyAssignedMessage;
    NoAvailableLobbyMessage noAvailableLobbyMessage;
    PlayMessage playMessage;
    MatchResultMessage matchResultMessage;
    QuitMessage quitMessage;
  };
};

struct Lobby {
  bool isPlaying;
  int player1Tid;
  int player2Tid;
  PlayType player1Play;
  PlayType player2Play;
};

} // namespace

namespace k2 {

void rpsServerTask() {
  // Register to the name server.
  RegisterAs(RPS_SERVER_NAME);

  RingBuffer<int, MAX_QUEUE_SIZE> playerQueue;
  Lobby lobbies[MAX_LOBBY] = {};

  // Handlers
  const auto signUpHandler = [&](int senderTid) {
    playerQueue.push(senderTid);
    if (playerQueue.size() < 2) {
      return;
    }
    int player1Tid = playerQueue.pop();
    int player2Tid = playerQueue.pop();

    RPSMessage msg;

    // Find the first available lobby.
    auto lobbyIt = kit::find_if(lobbies, &lobbies[MAX_LOBBY], [](const Lobby& lobby) { return !lobby.isPlaying; });

    if (lobbyIt == &lobbies[MAX_LOBBY]) {
      // No lobby available.
      msg = {.type = RPSMessageType::NO_AVAILABLE_LOBBY, .noAvailableLobbyMessage = NoAvailableLobbyMessage{}};

    } else {
      // Found a lobby.
      *lobbyIt = Lobby{true, player1Tid, player2Tid, PlayType::EMPTY, PlayType::EMPTY};
      int lobbyId = int(lobbyIt - lobbies);
      msg = {.type = RPSMessageType::LOBBY_ASSIGNED, .lobbyAssignedMessage = LobbyAssignedMessage{lobbyId}};
    }
    ::Reply(player1Tid, reinterpret_cast<const char*>(&msg), sizeof(RPSMessage));
    ::Reply(player2Tid, reinterpret_cast<const char*>(&msg), sizeof(RPSMessage));
  };

  const auto playHandler = [&](int senderTid, const PlayMessage& request) {
    Lobby& lobby = lobbies[request.lobbyId];

    // Player plays the move.
    if (lobby.player1Tid == senderTid) {
      lobby.player1Play = request.play;
    } else {
      lobby.player2Play = request.play;
    }

    // Both players have played their moves. Reply with the result.
    if (lobby.player1Play != PlayType::EMPTY && lobby.player2Play != PlayType::EMPTY) {
      RPSMessage reply = {.type = RPSMessageType::MATCH_RESULT,
                          .matchResultMessage = MatchResultMessage{lobby.player1Play, lobby.player2Play}};
      ::Reply(lobby.player1Tid, reinterpret_cast<const char*>(&reply), sizeof(RPSMessage));
      reply = {.type = RPSMessageType::MATCH_RESULT,
               .matchResultMessage = MatchResultMessage{lobby.player2Play, lobby.player1Play}};
      ::Reply(lobby.player2Tid, reinterpret_cast<const char*>(&reply), sizeof(RPSMessage));

      // Reset the lobby but continue playing.
      lobby.player1Play = PlayType::EMPTY;
      lobby.player2Play = PlayType::EMPTY;
    }
  };

  const auto quitHandler = [&](const QuitMessage& request) {
    Lobby& lobby = lobbies[request.lobbyId];
    lobby.isPlaying = false;
    RPSMessage reply = {.type = RPSMessageType::QUIT, .quitMessage = request};
    ::Reply(lobby.player1Tid, reinterpret_cast<const char*>(&reply), sizeof(RPSMessage));
    ::Reply(lobby.player2Tid, reinterpret_cast<const char*>(&reply), sizeof(RPSMessage));
  };

  // Handle all the requests.
  for (;;) {
    int senderTid;
    RPSMessage request;
    ::Receive(&senderTid, reinterpret_cast<char*>(&request), sizeof(RPSMessage));

    switch (request.type) {
    case RPSMessageType::SIGNUP:
      signUpHandler(senderTid);
      break;
    case RPSMessageType::PLAY:
      playHandler(senderTid, request.playMessage);
      break;
    case RPSMessageType::QUIT:
      quitHandler(request.quitMessage);
      break;
    default:
      logError("invalid and impossible client request");
    }
  }
}

void rpsClientTask() {
  int serverTid = WhoIs(RPS_SERVER_NAME);
  int lobbyId = 0;
  char buffer[128];

  // Enroll into the challenger queue.
  RPSMessage request = {.type = RPSMessageType::SIGNUP, .signUpMessage = SignUpMessage{}};
  RPSMessage reply;
  ::Send(serverTid, reinterpret_cast<const char*>(&request), sizeof(RPSMessage), reinterpret_cast<char*>(&reply),
         sizeof(RPSMessage));

  // No availabe lobby.
  if (reply.type == RPSMessageType::NO_AVAILABLE_LOBBY) {
    Uart::syncPrint(Uart::CONSOLE, "No available lobby! Please check again later.\r\n");
    return;
  }
  lobbyId = reply.lobbyAssignedMessage.lobbyId;
  kit::formatString(buffer,
                    "Lobby[%d] Enter your move [r]ock, [p]aper, [s]cissor or any other letter to quit: ", lobbyId);
  Uart::syncPrint(Uart::CONSOLE, buffer);

  // Play the move.
  PlayType play = [&buffer]() {
    char c = Uart::syncRead(Uart::CONSOLE);
    kit::formatString(buffer, "%c\r\n", c);
    Uart::syncPrint(Uart::CONSOLE, buffer);

    if (c == 'r' || c == 'R')
      return PlayType::ROCK;
    if (c == 'p' || c == 'P')
      return PlayType::PAPER;
    if (c == 's' || c == 'S')
      return PlayType::SCISSORS;
    return PlayType::EMPTY;
  }();

  // Player wants to quit.
  if (play == PlayType::EMPTY) {
    request = {.type = RPSMessageType::QUIT, .quitMessage = QuitMessage{lobbyId}};
    ::Send(serverTid, reinterpret_cast<const char*>(&request), sizeof(RPSMessage), reinterpret_cast<char*>(&reply),
           sizeof(RPSMessage));
    kit::formatString(buffer, "Lobby[%d] You quitted the game. The game is cancelled!\r\n", lobbyId);
    Uart::syncPrint(Uart::CONSOLE, buffer);
    return;
  }

  // Player wants to play.
  request = {.type = RPSMessageType::PLAY, .playMessage = PlayMessage{lobbyId, play}};
  ::Send(serverTid, reinterpret_cast<const char*>(&request), sizeof(RPSMessage), reinterpret_cast<char*>(&reply),
         sizeof(RPSMessage));

  // Opponent rage quitted.
  if (reply.type == RPSMessageType::QUIT) {
    kit::formatString(buffer, "Lobby[%d] Opponent quitted the game. The game is cancelled!\r\n", lobbyId);
    Uart::syncPrint(Uart::CONSOLE, buffer);
    return;
  }

  // Result;
  const char* playNames[] = {"rock", "paper", "scissors"};
  const char* result = [](PlayType me, PlayType you) {
    if (me == you) {
      return "draw";
    }
    switch (me) {
    case PlayType::ROCK:
      return (you == PlayType::SCISSORS) ? "win" : "lose";
    case PlayType::PAPER:
      return (you == PlayType::ROCK) ? "win" : "lose";
    case PlayType::SCISSORS:
      return (you == PlayType::PAPER) ? "win" : "lose";
    default:
      logError("impossible result");
    }
  }(reply.matchResultMessage.myPlay, reply.matchResultMessage.opponentPlay);

  kit::formatString(buffer, "Lobby[%d] You played %s, opponent played %s, result %s.\r\n", lobbyId,
                    playNames[int(reply.matchResultMessage.myPlay)],
                    playNames[int(reply.matchResultMessage.opponentPlay)], result);
  Uart::syncPrint(Uart::CONSOLE, buffer);

  // Quit the game.
  request = {.type = RPSMessageType::QUIT, .quitMessage = QuitMessage{lobbyId}};
  ::Send(serverTid, reinterpret_cast<const char*>(&request), sizeof(RPSMessage), reinterpret_cast<char*>(&reply),
         sizeof(RPSMessage));

  kit::formatString(buffer, "Lobby[%d] Game finished normally!\r\n", lobbyId);
  Uart::syncPrint(Uart::CONSOLE, buffer);
}

void FirstUserTask() {
  // Creates the name server.
  name_server::createNameServerTask(Priority::MEDIUM);

  // Creates the Rock/Paper/Scissors server.
  ::Create(Priority::MEDIUM, rpsServerTask);

  // Creates the Rock/Paper/Scissors clients.
  ::Create(Priority::MEDIUM, rpsClientTask);
  ::Create(Priority::MEDIUM, rpsClientTask);
}
} // namespace k2
