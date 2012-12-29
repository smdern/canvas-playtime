Behaviors =
  moveBall:
    execute: (sprite, context, dt) ->
      sprite.left += sprite.velocityX * dt
      sprite.top += sprite.velocityY * dt

