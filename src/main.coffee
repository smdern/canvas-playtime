$(document).ready ->

  targetCanvas = document.getElementById('canvas')
  game = new Playtime.Game(targetCanvas)

  game.start()

