# Iteratively read the file

readPopulation = function(filepath) {
  begin_iDV <- 0
  end_iDV <- 0
  begin_fDV <- 0
  end_fDV <- 0
  begin_obj <- 0
  end_obj <- 0
  begin_cnstrnt <- 0
  end_cnstrnt <- 0

  solutions <- vector("list", 0)

  con <- file(filepath, "r")
  while ( TRUE )
  {
    
    line <- readLines(con, n = 1)
    if ( length(line) == 0 )
    {
      break
    }
    
    # Split the string
    tok <- strsplit(line, "\\s+", fixed=FALSE)
    tok <- tok[[1]]

    # Locate the DVs
    # find begin iDV
    for (i in 1:length(tok))
    {
      if (tok[i] == "[")
      {
        begin_iDV <- i + 1
        break
      }
    }

    # Find end iDV
    for (i in begin_iDV:length(tok))
    {
      if (tok[i] == ";")
      {
        end_iDV <- i - 1
        begin_fDV <- i + 1
        break
      }
    }

    # find begin fDV
    for (i in end_iDV:length(tok))
    {
      if (tok[i] == "]")
      {
        end_fDV <- i - 1
        break
      }
    }

    # Locate the Locate the objs and constraints
    # Find begin obj
    for (i in end_fDV:length(tok))
    {
      if (tok[i] == "(")
      {
        begin_obj <- i + 1
        break
      }
    }

    # Find end obj
    for (i in begin_obj:length(tok))
    {
      if (tok[i] == ";")
      {
        end_obj <- i - 1
        begin_cnstrnt <- i + 1
        break
      }
    }

    # Find end constraint
    for (i in begin_cnstrnt:length(tok))
    {
      if (tok[i] == ")")
      {
        end_cnstrnt <- i - 1
        break
      }
    }

    
    if (begin_iDV < end_iDV)
    {
      iDV <- as.numeric(tok[begin_iDV:end_iDV])
    }
    else
    {
      iDV <- vector("integer", 0)
    }

    if (begin_fDV < end_fDV)
    {
      fDV <- as.numeric(tok[begin_fDV:end_fDV])
    }
    else
    {
      fDV <- vector("double", 0)
    }

    if (begin_obj < end_obj)
    {
      obj <- as.numeric(tok[begin_obj:end_obj])
    }
    else
    {
      obj <- vector("double", 0)
    }

    if (begin_cnstrnt < end_cnstrnt)
    {
      cnstrnt <- as.numeric(tok[begin_cnstrnt:end_cnstrnt])
    }
    else
    {
      cnstrnt <- vector("double", 0)
    }

    solution <- list(iDV, fDV, obj, cnstrnt)
    solutions[[length(solutions)+1]] <- solution
  }
  close(con)

  # Get Pareto front
  paretof <- vector("list", 0)
  for (j in 1:length(solutions[[1]][[3]]))
  {
    paretof[[j]] <- vector("double", 0)
  }
  for (i in 1:length(solutions))
  {
    for (j in 1:length(solutions[[i]][[3]]))
    {
       paretof[[j]] <- append(paretof[[j]], solutions[[i]][[3]][j])
    }
  }
  
  pareto_front_df <- data.frame(paretof)
  names_v <- vector(mode = "character")
  for (i in 1:length(paretof))
  {
    name <- paste("Obj", toString(i), sep = "")
    names_v[i] <- name
  }
  names(pareto_front_df) <- names_v
  

  return(list(solutions, pareto_front_df))
}





