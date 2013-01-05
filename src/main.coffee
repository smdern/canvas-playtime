$(document).ready ->

  PlayTime = {}

  targetCanvas = document.getElementById('canvas')
  PlayTime.game = new Game(targetCanvas)

  PlayTime.game.start()

