/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 *
 * The contents of this file are subject to the Netscape Public License
 * Version 1.0 (the "NPL"); you may not use this file except in
 * compliance with the NPL.  You may obtain a copy of the NPL at
 * http://www.mozilla.org/NPL/
 *
 * Software distributed under the NPL is distributed on an "AS IS" basis,
 * WITHOUT WARRANTY OF ANY KIND, either express or implied. See the NPL
 * for the specific language governing rights and limitations under the
 * NPL.
 *
 * The Initial Developer of this code under the NPL is Netscape
 * Communications Corporation.  Portions created by Netscape are
 * Copyright (C) 1998 Netscape Communications Corporation.  All Rights
 * Reserved.
 */
#include "nsTableRowGroupFrame.h"
#include "nsTableRowFrame.h"
#include "nsTableCellFrame.h"
#include "nsIRenderingContext.h"
#include "nsIPresContext.h"
#include "nsIStyleContext.h"
#include "nsStyleConsts.h"
#include "nsIContent.h"
#include "nsIView.h"
#include "nsIPtr.h"
#include "nsIReflowCommand.h"
#include "nsHTMLIIDs.h"

#ifdef NS_DEBUG
static PRBool gsDebug = PR_FALSE;
static PRBool gsDebugIR = PR_FALSE;
#else
static const PRBool gsDebug = PR_FALSE;
static const PRBool gsDebugIR = PR_FALSE;
#endif

NS_DEF_PTR(nsIStyleContext);
NS_DEF_PTR(nsIContent);

/* ----------- RowGroupReflowState ---------- */

struct RowGroupReflowState {
  nsIPresContext& mPresContext;  // Our pres context
  const nsHTMLReflowState& reflowState;  // Our reflow state

  // The body's available size (computed from the body's parent)
  nsSize availSize;

  // Margin tracking information
  nscoord prevMaxPosBottomMargin;
  nscoord prevMaxNegBottomMargin;

  // Flags for whether the max size is unconstrained
  PRBool  unconstrainedWidth;
  PRBool  unconstrainedHeight;

  // Running y-offset
  nscoord y;

  // Flag used to set maxElementSize to my first row
  PRBool  firstRow;

  // Remember the height of the first row, because it's our maxElementHeight (plus header/footers)
  nscoord firstRowHeight;

  RowGroupReflowState(nsIPresContext&          aPresContext,
                      const nsHTMLReflowState& aReflowState)
    : mPresContext(aPresContext),
      reflowState(aReflowState)
  {
    availSize.width = reflowState.maxSize.width;
    availSize.height = reflowState.maxSize.height;
    prevMaxPosBottomMargin = 0;
    prevMaxNegBottomMargin = 0;
    y=0;  // border/padding/margin???
    unconstrainedWidth = PRBool(reflowState.maxSize.width == NS_UNCONSTRAINEDSIZE);
    unconstrainedHeight = PRBool(reflowState.maxSize.height == NS_UNCONSTRAINEDSIZE);
    firstRow = PR_TRUE;
    firstRowHeight=0;
  }

  ~RowGroupReflowState() {
  }
};




/* ----------- nsTableRowGroupFrame ---------- */

nsTableRowGroupFrame::nsTableRowGroupFrame(nsIContent* aContent, nsIFrame* aParentFrame)
  : nsHTMLContainerFrame(aContent, aParentFrame)
{
  aContent->GetTag(mType);       // mType: REFCNT++
}

nsTableRowGroupFrame::~nsTableRowGroupFrame()
{
  NS_IF_RELEASE(mType);              // mType: REFCNT--
}

NS_METHOD nsTableRowGroupFrame::GetRowGroupType(nsIAtom *& aType)
{
  NS_ADDREF(mType);
  aType=mType;
  return NS_OK;
}

NS_METHOD nsTableRowGroupFrame::GetRowCount(PRInt32 &aCount)
{
  // init out-param
  aCount=0;

  // loop through children, adding one to aCount for every legit row
  nsIFrame *childFrame = mFirstChild;
  while (PR_TRUE)
  {
    if (nsnull==childFrame)
      break;
    const nsStyleDisplay *childDisplay;
    childFrame->GetStyleData(eStyleStruct_Display, ((nsStyleStruct *&)childDisplay));
    if (NS_STYLE_DISPLAY_TABLE_ROW == childDisplay->mDisplay)
      aCount++;
    childFrame->GetNextSibling(childFrame);
  }
  return NS_OK;
}

NS_METHOD nsTableRowGroupFrame::GetMaxColumns(PRInt32 &aMaxColumns) const
{
  aMaxColumns=0;
  // loop through children, remembering the max of the columns in each row
  nsIFrame *childFrame = mFirstChild;
  while (PR_TRUE)
  {
    if (nsnull==childFrame)
      break;
    const nsStyleDisplay *childDisplay;
    childFrame->GetStyleData(eStyleStruct_Display, ((nsStyleStruct *&)childDisplay));
    if (NS_STYLE_DISPLAY_TABLE_ROW == childDisplay->mDisplay)
    {
      PRInt32 colCount = ((nsTableRowFrame *)childFrame)->GetMaxColumns();
      aMaxColumns = PR_MAX(aMaxColumns, colCount);
    }
    childFrame->GetNextSibling(childFrame);
  }
  return NS_OK;
}


NS_IMETHODIMP
nsTableRowGroupFrame::Init(nsIPresContext& aPresContext, nsIFrame* aChildList)
{
  mFirstChild = aChildList;
  return NS_OK;
}

NS_METHOD nsTableRowGroupFrame::Paint(nsIPresContext& aPresContext,
                                      nsIRenderingContext& aRenderingContext,
                                      const nsRect&        aDirtyRect)
{

  // for debug...
  /*
  if (nsIFrame::GetShowFrameBorders()) {
    aRenderingContext.SetColor(NS_RGB(128,0,0));
    aRenderingContext.DrawRect(0, 0, mRect.width, mRect.height);
  }
  */

  PaintChildren(aPresContext, aRenderingContext, aDirtyRect);
  return NS_OK;
}

PRIntn
nsTableRowGroupFrame::GetSkipSides() const
{
  PRIntn skip = 0;
  if (nsnull != mPrevInFlow) {
    skip |= 1 << NS_SIDE_TOP;
  }
  if (nsnull != mNextInFlow) {
    skip |= 1 << NS_SIDE_BOTTOM;
  }
  return skip;
}

// aDirtyRect is in our coordinate system
// child rect's are also in our coordinate system
/** overloaded method from nsContainerFrame.  The difference is that 
  * we don't want to clip our children, so a cell can do a rowspan
  */
void nsTableRowGroupFrame::PaintChildren(nsIPresContext&      aPresContext,
                                         nsIRenderingContext& aRenderingContext,
                                         const nsRect&        aDirtyRect)
{
  nsIFrame* kid = mFirstChild;
  while (nsnull != kid) {
    nsIView *pView;
     
    kid->GetView(pView);
    if (nsnull == pView) {
      PRBool clipState;
      nsRect kidRect;
      kid->GetRect(kidRect);
      nsRect damageArea(aDirtyRect);
      // Translate damage area into kid's coordinate system
      nsRect kidDamageArea(damageArea.x - kidRect.x, damageArea.y - kidRect.y,
                           damageArea.width, damageArea.height);
      aRenderingContext.PushState();
      aRenderingContext.Translate(kidRect.x, kidRect.y);
      kid->Paint(aPresContext, aRenderingContext, kidDamageArea);
      if (nsIFrame::GetShowFrameBorders()) {
        aRenderingContext.SetColor(NS_RGB(255,0,0));
        aRenderingContext.DrawRect(0, 0, kidRect.width, kidRect.height);
      }
      aRenderingContext.PopState(clipState);
    }
    kid->GetNextSibling(kid);
  }
}

// Collapse child's top margin with previous bottom margin
nscoord nsTableRowGroupFrame::GetTopMarginFor(nsIPresContext*      aCX,
                                              RowGroupReflowState& aReflowState,
                                              const nsMargin& aKidMargin)
{
  nscoord margin;
  nscoord maxNegTopMargin = 0;
  nscoord maxPosTopMargin = 0;
  if ((margin = aKidMargin.top) < 0) {
    maxNegTopMargin = -margin;
  } else {
    maxPosTopMargin = margin;
  }

  nscoord maxPos = PR_MAX(aReflowState.prevMaxPosBottomMargin, maxPosTopMargin);
  nscoord maxNeg = PR_MAX(aReflowState.prevMaxNegBottomMargin, maxNegTopMargin);
  margin = maxPos - maxNeg;

  return margin;
}

// Position and size aKidFrame and update our reflow state. The origin of
// aKidRect is relative to the upper-left origin of our frame, and includes
// any left/top margin.
void nsTableRowGroupFrame::PlaceChild(nsIPresContext&      aPresContext,
																			RowGroupReflowState& aReflowState,
																			nsIFrame*            aKidFrame,
																			const nsRect&        aKidRect,
																			nsSize*              aMaxElementSize,
																			nsSize&              aKidMaxElementSize)
{
  if (PR_TRUE==gsDebug)
    printf ("rowgroup %p: placing row at %d, %d, %d, %d\n",
            this, aKidRect.x, aKidRect.y, aKidRect.width, aKidRect.height);

  // Place and size the child
  aKidFrame->SetRect(aKidRect);

  // Adjust the running y-offset
  aReflowState.y += aKidRect.height;

  // If our height is constrained then update the available height
  if (PR_FALSE == aReflowState.unconstrainedHeight) {
    aReflowState.availSize.height -= aKidRect.height;
  }

  // Update the maximum element size
  if (PR_TRUE==aReflowState.firstRow)
  {
    aReflowState.firstRow = PR_FALSE;
    aReflowState.firstRowHeight = aKidRect.height;
    if (nsnull != aMaxElementSize) {
      aMaxElementSize->width = aKidMaxElementSize.width;
      aMaxElementSize->height = aKidMaxElementSize.height;
    }
  }
  else if (nsnull != aMaxElementSize) {
      aMaxElementSize->width = PR_MAX(aMaxElementSize->width, aKidMaxElementSize.width);
  }
  if (gsDebug && nsnull != aMaxElementSize)
    printf ("rowgroup %p: placing row %p with width = %d and MES= %d\n",
            this, aKidFrame, aKidRect.width, aMaxElementSize->width);
}

/**
 * Reflow the frames we've already created
 *
 * @param   aPresContext presentation context to use
 * @param   aReflowState current inline state
 * @return  true if we successfully reflowed all the mapped children and false
 *            otherwise, e.g. we pushed children to the next in flow
 */
NS_METHOD nsTableRowGroupFrame::ReflowMappedChildren(nsIPresContext&      aPresContext,
                                                     nsHTMLReflowMetrics& aDesiredSize,
                                                     RowGroupReflowState& aReflowState,
                                                     nsReflowStatus&      aStatus,
                                                     nsTableRowFrame *    aStartFrame,
                                                     nsReflowReason       aReason,
                                                     PRBool               aDoSiblings)
{
  if (PR_TRUE==gsDebugIR) printf("\nTRGF IR: ReflowMappedChildren\n");
  nsIFrame* prevKidFrame = nsnull;
  nsSize    kidMaxElementSize;
  nsSize*   pKidMaxElementSize = (nsnull != aDesiredSize.maxElementSize) ? &kidMaxElementSize : nsnull;
  nsresult  rv = NS_OK;
  nsIFrame*  kidFrame;
  if (nsnull==aStartFrame)
    kidFrame = mFirstChild;
  else
    kidFrame = aStartFrame;
  for ( ; nsnull != kidFrame; ) 
  {
    nsSize kidAvailSize(aReflowState.availSize);
    if (0>=kidAvailSize.height)
      kidAvailSize.height = 1;      // XXX: HaCk - we don't handle negative heights yet
    nsHTMLReflowMetrics desiredSize(pKidMaxElementSize);
    desiredSize.width=desiredSize.height=desiredSize.ascent=desiredSize.descent=0;

    // Get top margin for this kid
    const nsStyleSpacing* kidSpacing;
    kidFrame->GetStyleData(eStyleStruct_Spacing, (const nsStyleStruct *&)kidSpacing);
    nsMargin kidMargin;
    kidSpacing->CalcMarginFor(this, kidMargin);
    nscoord topMargin = GetTopMarginFor(&aPresContext, aReflowState, kidMargin);
    nscoord bottomMargin = kidMargin.bottom;

    // Figure out the amount of available size for the child (subtract
    // off the top margin we are going to apply to it)
    if (PR_FALSE == aReflowState.unconstrainedHeight) {
      kidAvailSize.height -= topMargin;
    }
    // Subtract off for left and right margin
    if (PR_FALSE == aReflowState.unconstrainedWidth) {
      kidAvailSize.width -= kidMargin.left + kidMargin.right;
    }

    // Reflow the child into the available space
    nsHTMLReflowState kidReflowState(aPresContext, kidFrame, aReflowState.reflowState,
                                     kidAvailSize, aReason);

    if ((PR_TRUE==gsDebug) || (PR_TRUE==gsDebugIR))
      printf("%p RG reflowing child %p with avail width = %d, reason = %d\n",
                        this, kidFrame, kidAvailSize.width, aReason);
    rv = ReflowChild(kidFrame, aPresContext, desiredSize, kidReflowState, aStatus);
    if (gsDebug) printf("%p RG child %p returned desired width = %d\n",
                        this, kidFrame, desiredSize.width);

    // Did the child fit?
    if ((kidFrame != mFirstChild) &&
        ((kidAvailSize.height <= 0) ||
         (desiredSize.height > kidAvailSize.height)))
    {
      // The child's height is too big to fit at all in our remaining space,
      // and it's not our first child.
      //
      // Note that if the width is too big that's okay and we allow the
      // child to extend horizontally outside of the reflow area
      PushChildren(kidFrame, prevKidFrame);
      break;
    }

    // Place the child after taking into account it's margin
    nsRect kidRect (kidMargin.left, aReflowState.y, desiredSize.width, desiredSize.height);
    PlaceChild(aPresContext, aReflowState, kidFrame, kidRect, aDesiredSize.maxElementSize,
               kidMaxElementSize);
    if (bottomMargin < 0) {
      aReflowState.prevMaxNegBottomMargin = -bottomMargin;
    } else {
      aReflowState.prevMaxPosBottomMargin = bottomMargin;
    }

		// Remember where we just were in case we end up pushing children
		prevKidFrame = kidFrame;

    /* Row groups should not create continuing frames for rows 
     * unless they absolutely have to!
     * check to see if this is absolutely necessary (with new params from troy)
     * otherwise PushChildren and bail.
     */
    // Special handling for incomplete children
    if (NS_FRAME_IS_NOT_COMPLETE(aStatus)) {
      // XXX It's good to assume that we might still have room
      // even if the child didn't complete (floaters will want this)
      nsIFrame* kidNextInFlow;
       
      kidFrame->GetNextInFlow(kidNextInFlow);
      if (nsnull == kidNextInFlow) {
        // No the child isn't complete, and it doesn't have a next in flow so
        // create a continuing frame. This hooks the child into the flow.
        nsIFrame* continuingFrame;
        nsIStyleContext* kidSC;
        kidFrame->GetStyleContext(&aPresContext, kidSC);
        kidFrame->CreateContinuingFrame(aPresContext, this, kidSC,
                                        continuingFrame);
        NS_RELEASE(kidSC);

        // Insert the frame. We'll reflow it next pass through the loop
        nsIFrame* nextSib;
         
        kidFrame->GetNextSibling(nextSib);
        continuingFrame->SetNextSibling(nextSib);
        kidFrame->SetNextSibling(continuingFrame);
      }
      // We've used up all of our available space so push the remaining
      // children to the next-in-flow
      nsIFrame* nextSibling;
       
      kidFrame->GetNextSibling(nextSibling);
      if (nsnull != nextSibling) {
        PushChildren(nextSibling, kidFrame);
      }
      break;
    }

    // Add back in the left and right margins, because one row does not 
    // impact another row's width
    if (PR_FALSE == aReflowState.unconstrainedWidth) {
      kidAvailSize.width += kidMargin.left + kidMargin.right;
    }

    if (PR_FALSE==aDoSiblings)
      break;

    // Get the next child
    kidFrame->GetNextSibling(kidFrame);
  }

  return rv;
}

/**
 * Try and pull-up frames from our next-in-flow
 *
 * @param   aPresContext presentation context to use
 * @param   aReflowState current inline state
 * @return  true if we successfully pulled-up all the children and false
 *            otherwise, e.g. child didn't fit
 */
NS_METHOD nsTableRowGroupFrame::PullUpChildren(nsIPresContext&      aPresContext,
                                               nsHTMLReflowMetrics& aDesiredSize,
                                               RowGroupReflowState& aReflowState,
                                               nsReflowStatus&      aStatus)
{
  nsTableRowGroupFrame* nextInFlow = (nsTableRowGroupFrame*)mNextInFlow;
  nsSize         kidMaxElementSize;
  nsSize*        pKidMaxElementSize = (nsnull != aDesiredSize.maxElementSize) ? &kidMaxElementSize : nsnull;
  nsIFrame*      prevKidFrame = LastFrame(mFirstChild);
  nsresult       rv = NS_OK;

  while (nsnull != nextInFlow) {
    nsHTMLReflowMetrics kidSize(pKidMaxElementSize);
    kidSize.width=kidSize.height=kidSize.ascent=kidSize.descent=0;

    // Get the next child
    nsIFrame* kidFrame = nextInFlow->mFirstChild;

    // Any more child frames?
    if (nsnull == kidFrame) {
      // No. Any frames on its overflow list?
      if (nsnull != nextInFlow->mOverflowList) {
        // Move the overflow list to become the child list
//        NS_ABORT();
        nextInFlow->AppendChildren(nextInFlow->mOverflowList);
        nextInFlow->mOverflowList = nsnull;
        kidFrame = nextInFlow->mFirstChild;
      } else {
        // We've pulled up all the children, so move to the next-in-flow.
        nextInFlow->GetNextInFlow((nsIFrame*&)nextInFlow);
        continue;
      }
    }

    // See if the child fits in the available space. If it fits or
    // it's splittable then reflow it. The reason we can't just move
    // it is that we still need ascent/descent information
    nsSize            kidFrameSize;
    nsSplittableType  kidIsSplittable;

    kidFrame->GetSize(kidFrameSize);
    kidFrame->IsSplittable(kidIsSplittable);
    if ((kidFrameSize.height > aReflowState.availSize.height) &&
        NS_FRAME_IS_NOT_SPLITTABLE(kidIsSplittable)) {
      break;
    }
    nsHTMLReflowState kidReflowState(aPresContext, kidFrame,
                                     aReflowState.reflowState, aReflowState.availSize,
                                     eReflowReason_Resize);

    rv = ReflowChild(kidFrame, aPresContext, kidSize, kidReflowState, aStatus);

    // Did the child fit?
    if ((kidSize.height > aReflowState.availSize.height) && (nsnull != mFirstChild)) {
      // The child is too wide to fit in the available space, and it's
      // not our first child
      //result = PR_FALSE;
      break;
    }

    // Place the child
    //aReflowState.y += topMargin;
    nsRect kidRect (0, 0, kidSize.width, kidSize.height);
    //kidRect.x += kidMol->margin.left;
    kidRect.y += aReflowState.y;
    PlaceChild(aPresContext, aReflowState, kidFrame, kidRect, aDesiredSize.maxElementSize, *pKidMaxElementSize);

    // Remove the frame from its current parent
    kidFrame->GetNextSibling(nextInFlow->mFirstChild);

    // Link the frame into our list of children
    kidFrame->SetGeometricParent(this);
    nsIFrame* kidContentParent;

    kidFrame->GetContentParent(kidContentParent);
    if (nextInFlow == kidContentParent) {
      kidFrame->SetContentParent(this);
    }
    if (nsnull == prevKidFrame) {
      mFirstChild = kidFrame;
    } else {
      prevKidFrame->SetNextSibling(kidFrame);
    }
    kidFrame->SetNextSibling(nsnull);

    // Remember where we just were in case we end up pushing children
    prevKidFrame = kidFrame;

    // Is the child we just pulled up complete?
    if (NS_FRAME_IS_NOT_COMPLETE(aStatus)) {
      // No the child isn't complete
      nsIFrame* kidNextInFlow;
       
      kidFrame->GetNextInFlow(kidNextInFlow);
      if (nsnull == kidNextInFlow) {
        // The child doesn't have a next-in-flow so create a
        // continuing frame. The creation appends it to the flow and
        // prepares it for reflow.
        nsIFrame* continuingFrame;
        nsIStyleContext* kidSC;
        kidFrame->GetStyleContext(&aPresContext, kidSC);
        kidFrame->CreateContinuingFrame(aPresContext, this, kidSC,
                                        continuingFrame);
        NS_RELEASE(kidSC);
        NS_ASSERTION(nsnull != continuingFrame, "frame creation failed");

        // Add the continuing frame to our sibling list and then push
        // it to the next-in-flow. This ensures the next-in-flow's
        // content offsets and child count are set properly. Note that
        // we can safely assume that the continuation is complete so
        // we pass PR_TRUE into PushChidren
        kidFrame->SetNextSibling(continuingFrame);

        PushChildren(continuingFrame, kidFrame);
      }

      // If the child isn't complete then it means that we've used up
      // all of our available space.
      //result = PR_FALSE;
      break;
    }
  }

  return rv;
}

/**
  */
void nsTableRowGroupFrame::ShrinkWrapChildren(nsIPresContext* aPresContext, 
                                              nsHTMLReflowMetrics& aDesiredSize)
{
  // iterate children and for each row get its height
  PRBool atLeastOneRowSpanningCell = PR_FALSE;
  nscoord topInnerMargin = 0;
  nscoord bottomInnerMargin = 0;
  PRInt32 numRows;
  GetRowCount(numRows);
  PRInt32 *rowHeights = new PRInt32[numRows];
  nsCRT::memset (rowHeights, 0, numRows*sizeof(PRInt32));

  /* Step 1:  get the height of the tallest cell in the row and save it for
   *          pass 2
   */
  nsIFrame* rowFrame = mFirstChild;
  PRInt32 rowIndex = 0;
  while (nsnull != rowFrame)
  {
    const nsStyleDisplay *childDisplay;
    rowFrame->GetStyleData(eStyleStruct_Display, ((nsStyleStruct *&)childDisplay));
    if (NS_STYLE_DISPLAY_TABLE_ROW == childDisplay->mDisplay)
    {
      // get the height of the tallest cell in the row (excluding cells that span rows)
      nscoord maxCellHeight       = ((nsTableRowFrame*)rowFrame)->GetTallestChild();
      nscoord maxCellTopMargin    = ((nsTableRowFrame*)rowFrame)->GetChildMaxTopMargin();
      nscoord maxCellBottomMargin = ((nsTableRowFrame*)rowFrame)->GetChildMaxBottomMargin();
      nscoord maxRowHeight = maxCellHeight + maxCellTopMargin + maxCellBottomMargin;

      // save the row height for pass 2 below
      rowHeights[rowIndex] = maxRowHeight;

      // Update top and bottom inner margin if applicable
      if (0 == rowIndex) {
        topInnerMargin = maxCellTopMargin;
      }
      if ((rowIndex + 1) == numRows) {
        bottomInnerMargin = maxCellBottomMargin;
      }
      rowIndex++;
    }
    // Get the next row
    rowFrame->GetNextSibling(rowFrame);
  }

  /* Step 2:  now account for cells that span rows.
   *          a spanning cell's height is the sum of the heights of the rows it spans,
   *          or it's own desired height, whichever is greater.
   *          If the cell's desired height is the larger value, resize the rows and contained
   *          cells by an equal percentage of the additional space.
   */
  /* TODO
   * 1. optimization, if (PR_TRUE==atLeastOneRowSpanningCell) ... otherwise skip this step entirely
   * 2. find cases where spanning cells effect other spanning cells that began in rows above themselves.
   *    I think in this case, we have to make another pass through step 2.
   *    There should be a "rational" check to terminate that kind of loop after n passes, probably 3 or 4.
   */
  PRInt32 rowGroupHeight = 0;
  rowFrame = mFirstChild;
  rowIndex = 0;
  while (nsnull != rowFrame)
  {
    const nsStyleDisplay *childDisplay;
    rowFrame->GetStyleData(eStyleStruct_Display, ((nsStyleStruct *&)childDisplay));
    if (NS_STYLE_DISPLAY_TABLE_ROW == childDisplay->mDisplay)
    {
      // check this row for a cell with rowspans
      nsIFrame *cellFrame;
      rowFrame->FirstChild(cellFrame);
      while (nsnull != cellFrame)
      {
        const nsStyleDisplay *childDisplay;
        cellFrame->GetStyleData(eStyleStruct_Display, ((nsStyleStruct *&)childDisplay));
        if (NS_STYLE_DISPLAY_TABLE_CELL == childDisplay->mDisplay)
        {
          nsTableFrame *tableFrame=nsnull;
          nsresult rv = nsTableFrame::GetTableFrame(this, tableFrame);
          if (NS_FAILED(rv) || nsnull==tableFrame)
            return;
          PRInt32 rowSpan = tableFrame->GetEffectiveRowSpan(rowIndex,(nsTableCellFrame*)cellFrame);
          if (rowSpan > 1)
          { // found a cell with rowspan > 1, determine its height
            nscoord heightOfRowsSpanned = 0;
            PRInt32 i;
            for ( i = 0; i < rowSpan; i++)
              heightOfRowsSpanned += rowHeights[rowIndex + i];
        
            heightOfRowsSpanned -= topInnerMargin + bottomInnerMargin;

            /* if the cell height fits in the rows, expand the spanning cell's height and slap it in */
            nsSize  cellFrameSize;
            cellFrame->GetSize(cellFrameSize);
            if (heightOfRowsSpanned > cellFrameSize.height)
            {
              cellFrame->SizeTo(cellFrameSize.width, heightOfRowsSpanned);
              // Realign cell content based on new height
              ((nsTableCellFrame*)cellFrame)->VerticallyAlignChild(aPresContext);
            }
            /* otherwise, distribute the excess height to the rows effected.
             * push all subsequent rows down by the total change in height of all the rows above it
             */
            else
            {
              PRInt32 excessHeight = cellFrameSize.height - heightOfRowsSpanned;
              PRInt32 excessHeightPerRow = excessHeight/rowSpan;

              // for every row starting at the row with the spanning cell...
              nsTableRowFrame *rowFrameToBeResized = (nsTableRowFrame *)rowFrame;
              for (i = rowIndex; i < numRows; i++)
              {
                // if the row is within the spanned range, resize the row
                if (i < (rowIndex + rowSpan))
                {
                  // update the row height
                  rowHeights[i] += excessHeightPerRow;

                  // adjust the height of the row
                  nsSize  rowFrameSize;
                  rowFrameToBeResized->GetSize(rowFrameSize);
                  rowFrameToBeResized->SizeTo(rowFrameSize.width, rowHeights[i]);
                }

                // if we're dealing with a row below the row containing the spanning cell, 
                // push that row down by the amount we've expanded the cell heights by
                if ((i >= rowIndex) && (i != 0))
                {
                  nsRect rowRect;
               
                  rowFrameToBeResized->GetRect(rowRect);
                  nscoord delta = excessHeightPerRow * (i - rowIndex);
                  if (delta > excessHeight)
                    delta = excessHeight;
                  rowFrameToBeResized->MoveTo(rowRect.x, rowRect.y + delta);
                }

                // Get the next row frame
                rowFrameToBeResized->GetNextSibling((nsIFrame*&)rowFrameToBeResized);
              }
            }
          }
        }
        // Get the next row child (cell frame)
        cellFrame->GetNextSibling(cellFrame);
      }
      // Update the running row group height
      rowGroupHeight += rowHeights[rowIndex];
      rowIndex++;
    }
    // Get the next rowgroup child (row frame)
    rowFrame->GetNextSibling(rowFrame);
  }

  // Adjust our desired size
  aDesiredSize.height = rowGroupHeight;

  // cleanup
  delete []rowHeights;
}

nsresult nsTableRowGroupFrame::AdjustSiblingsAfterReflow(nsIPresContext&      aPresContext,
                                                         RowGroupReflowState& aReflowState,
                                                         nsIFrame*            aKidFrame,
                                                         nscoord              aDeltaY)
{
  if (PR_TRUE==gsDebugIR) printf("\nTRGF IR: AdjustSiblingsAfterReflow\n");
  nsIFrame* lastKidFrame = aKidFrame;

  if (aDeltaY != 0) {
    // Move the frames that follow aKidFrame by aDeltaY
    nsIFrame* kidFrame;

    aKidFrame->GetNextSibling(kidFrame);
    while (nsnull != kidFrame) {
      nsPoint origin;
  
      // XXX We can't just slide the child if it has a next-in-flow
      kidFrame->GetOrigin(origin);
      origin.y += aDeltaY;
  
      // XXX We need to send move notifications to the frame...
      nsIHTMLReflow* htmlReflow;
      if (NS_OK == kidFrame->QueryInterface(kIHTMLReflowIID, (void**)&htmlReflow)) {
        htmlReflow->WillReflow(aPresContext);
      }
      kidFrame->MoveTo(origin.x, origin.y);

      // Get the next frame
      lastKidFrame = kidFrame;
      kidFrame->GetNextSibling(kidFrame);
    }

  } else {
    // Get the last frame
    lastKidFrame = LastFrame(mFirstChild);
  }

  // XXX Deal with cells that have rowspans.

  // Update our running y-offset to reflect the bottommost child
  nsRect  rect;
  lastKidFrame->GetRect(rect);
  aReflowState.y = rect.YMost();

  return NS_OK;
}

/** Layout the entire row group.
  * This method stacks rows vertically according to HTML 4.0 rules.
  * Rows are responsible for layout of their children.
  */
NS_METHOD
nsTableRowGroupFrame::Reflow(nsIPresContext&          aPresContext,
                             nsHTMLReflowMetrics&     aDesiredSize,
                             const nsHTMLReflowState& aReflowState,
                             nsReflowStatus&          aStatus)
{
  nsresult rv=NS_OK;
  if (gsDebug==PR_TRUE)
    printf("nsTableRowGroupFrame::Reflow - aMaxSize = %d, %d\n",
            aReflowState.maxSize.width, aReflowState.maxSize.height);

  // Initialize out parameter
  if (nsnull != aDesiredSize.maxElementSize) {
    aDesiredSize.maxElementSize->width = 0;
    aDesiredSize.maxElementSize->height = 0;
  }

  RowGroupReflowState state(aPresContext, aReflowState);

  if (eReflowReason_Incremental == aReflowState.reason) {
    rv = IncrementalReflow(aPresContext, aDesiredSize, aReflowState, aStatus);
  } else {
    PRBool reflowMappedOK = PR_TRUE;
  
    aStatus = NS_FRAME_COMPLETE;
  
    // Check for an overflow list
    MoveOverflowToChildList();
  
    // Reflow the existing frames
    if (nsnull != mFirstChild) {
      rv = ReflowMappedChildren(aPresContext, aDesiredSize, state, aStatus,
                                nsnull, aReflowState.reason, PR_TRUE);
    }
  
    // Did we successfully reflow our mapped children?
    if (NS_FRAME_COMPLETE==aStatus) {
      // Any space left?
      PRInt32 numKids;
      mContent->ChildCount(numKids);
      if (state.availSize.height > 0) {
        // Try and pull-up some children from a next-in-flow
        rv = PullUpChildren(aPresContext, aDesiredSize, state, aStatus);
      }
    }
  
    if (NS_FRAME_IS_COMPLETE(aStatus)) {
      // Don't forget to add in the bottom margin from our last child.
      // Only add it in if there's room for it.
      nscoord margin = state.prevMaxPosBottomMargin -
        state.prevMaxNegBottomMargin;
      if (state.availSize.height >= margin) {
        state.y += margin;
      }
    }
  
    // Return our desired rect
    aDesiredSize.width = aReflowState.maxSize.width;
    aDesiredSize.height = state.y;

    // shrink wrap rows to height of tallest cell in that row
    ShrinkWrapChildren(&aPresContext, aDesiredSize);
  }

  if (gsDebug==PR_TRUE) 
  {
    if (nsnull!=aDesiredSize.maxElementSize)
      printf("nsTableRowGroupFrame %p returning: %s with aDesiredSize=%d,%d, aMES=%d,%d\n",
              this, NS_FRAME_IS_COMPLETE(aStatus)?"Complete":"Not Complete",
              aDesiredSize.width, aDesiredSize.height,
              aDesiredSize.maxElementSize->width, aDesiredSize.maxElementSize->height);
    else
      printf("nsTableRowGroupFrame %p returning: %s with aDesiredSize=%d,%d, aMES=NSNULL\n", 
             this, NS_FRAME_IS_COMPLETE(aStatus)?"Complete":"Not Complete",
             aDesiredSize.width, aDesiredSize.height);
  }
  return rv;
}


NS_METHOD nsTableRowGroupFrame::IncrementalReflow(nsIPresContext& aPresContext,
                                                  nsHTMLReflowMetrics& aDesiredSize,
                                                  const nsHTMLReflowState& aReflowState,
                                                  nsReflowStatus& aStatus)
{
  if (PR_TRUE==gsDebugIR) printf("\nTRGF IR: IncrementalReflow\n");
  nsresult  rv = NS_OK;

  // create an inner table reflow state
  RowGroupReflowState state(aPresContext, aReflowState);

  // determine if this frame is the target or not
  nsIFrame *target=nsnull;
  rv = aReflowState.reflowCommand->GetTarget(target);
  if ((PR_TRUE==NS_SUCCEEDED(rv)) && (nsnull!=target))
  {
    if (this==target)
      rv = IR_TargetIsMe(aPresContext, aDesiredSize, state, aStatus);
    else
    {
      // Get the next frame in the reflow chain
      nsIFrame* nextFrame;
      aReflowState.reflowCommand->GetNext(nextFrame);

      // Recover our reflow state
      //RecoverState(state, nextFrame);
      rv = IR_TargetIsChild(aPresContext, aDesiredSize, state, aStatus, nextFrame);
    }
  }
  return rv;
}

NS_METHOD nsTableRowGroupFrame::IR_TargetIsMe(nsIPresContext&      aPresContext,
                                              nsHTMLReflowMetrics& aDesiredSize,
                                              RowGroupReflowState& aReflowState,
                                              nsReflowStatus&      aStatus)
{
  nsresult rv = NS_FRAME_COMPLETE;
  nsIReflowCommand::ReflowType type;
  aReflowState.reflowState.reflowCommand->GetType(type);
  nsIFrame *objectFrame;
  aReflowState.reflowState.reflowCommand->GetChildFrame(objectFrame); 
  const nsStyleDisplay *childDisplay=nsnull;
  if (nsnull!=objectFrame)
    objectFrame->GetStyleData(eStyleStruct_Display, ((nsStyleStruct *&)childDisplay));
  if (PR_TRUE==gsDebugIR) printf("TRGF IR: TargetIsMe with type=%d\n", type);
  switch (type)
  {
  case nsIReflowCommand::FrameInserted :
    NS_ASSERTION(nsnull!=objectFrame, "bad objectFrame");
    NS_ASSERTION(nsnull!=childDisplay, "bad childDisplay");
    if (NS_STYLE_DISPLAY_TABLE_ROW == childDisplay->mDisplay)
    {
      rv = IR_RowInserted(aPresContext, aDesiredSize, aReflowState, aStatus, 
                          (nsTableRowFrame *)objectFrame, PR_FALSE);
    }
    else
    {
      rv = AddFrame(aReflowState.reflowState, objectFrame);
    }
    break;
  
  case nsIReflowCommand::FrameAppended :
    NS_ASSERTION(nsnull!=objectFrame, "bad objectFrame");
    NS_ASSERTION(nsnull!=childDisplay, "bad childDisplay");
    if (NS_STYLE_DISPLAY_TABLE_ROW == childDisplay->mDisplay)
    {
      rv = IR_RowAppended(aPresContext, aDesiredSize, aReflowState, aStatus, 
                          (nsTableRowFrame *)objectFrame);
    }
    else
    { // no optimization to be done for Unknown frame types, so just reuse the Inserted method
      rv = AddFrame(aReflowState.reflowState, objectFrame);
    }
    break;

  /*
  case nsIReflowCommand::FrameReplaced :

  */

  case nsIReflowCommand::FrameRemoved :
    NS_ASSERTION(nsnull!=objectFrame, "bad objectFrame");
    NS_ASSERTION(nsnull!=childDisplay, "bad childDisplay");
    if (NS_STYLE_DISPLAY_TABLE_ROW == childDisplay->mDisplay)
    {
      rv = IR_RowRemoved(aPresContext, aDesiredSize, aReflowState, aStatus, 
                         (nsTableRowFrame *)objectFrame);
    }
    else
    {
      rv = RemoveFrame(objectFrame);
    }
    break;

  case nsIReflowCommand::StyleChanged :
    rv = IR_StyleChanged(aPresContext, aDesiredSize, aReflowState, aStatus);
    break;

  case nsIReflowCommand::ContentChanged :
    NS_ASSERTION(PR_FALSE, "illegal reflow type: ContentChanged");
    rv = NS_ERROR_ILLEGAL_VALUE;
    break;
  
  case nsIReflowCommand::PullupReflow:
  case nsIReflowCommand::PushReflow:
  case nsIReflowCommand::CheckPullupReflow :
  case nsIReflowCommand::UserDefined :
    NS_NOTYETIMPLEMENTED("unimplemented reflow command type");
    rv = NS_ERROR_NOT_IMPLEMENTED;
    if (PR_TRUE==gsDebugIR) printf("TRGF IR: reflow command not implemented.\n");
    break;
  }

  return rv;
}

NS_METHOD nsTableRowGroupFrame::IR_RowInserted(nsIPresContext&      aPresContext,
                                               nsHTMLReflowMetrics& aDesiredSize,
                                               RowGroupReflowState& aReflowState,
                                               nsReflowStatus&      aStatus,
                                               nsTableRowFrame *    aInsertedFrame,
                                               PRBool               aReplace)
{
  if (PR_TRUE==gsDebugIR) printf("\nTRGF IR: IR_RowInserted\n");
  nsresult rv = AddFrame(aReflowState.reflowState, (nsIFrame*)aInsertedFrame);
  if (NS_FAILED(rv))
    return rv;

  nsTableFrame *tableFrame=nsnull;
  rv = nsTableFrame::GetTableFrame(this, tableFrame);
  if (NS_FAILED(rv) || nsnull==tableFrame)
    return rv;

  if (PR_TRUE==tableFrame->RequiresPass1Layout())
  {
    // do a pass-1 layout of all the cells in the inserted row
    //XXX: check the table frame to see if we can skip this
    rv = ReflowMappedChildren(aPresContext, aDesiredSize, aReflowState, aStatus, 
                              aInsertedFrame, eReflowReason_Initial, PR_FALSE);
    if (NS_FAILED(rv))
      return rv;
  }

  tableFrame->InvalidateCellMap();
  tableFrame->InvalidateColumnCache();

  return rv;
}

NS_METHOD nsTableRowGroupFrame::DidAppendRow(nsTableRowFrame *aRowFrame)
{
  if (PR_TRUE==gsDebugIR) printf("\nTRGF IR: DidAppendRow\n");
  nsresult rv=NS_OK;
  /* need to make space in the cell map.  Remeber that row spans can't cross row groups
     once the space is made, tell the row to initizalize its children.  
     it will automatically find the row to initialize into.
     but this is tough because a cell in aInsertedFrame could have a rowspan
     which must be respected if a subsequent row is appended.
  */
  rv = aRowFrame->InitChildren();
  return rv;
}

// support method that tells us if there are any rows in the table after our rows
// returns PR_TRUE if there are no rows after ours
PRBool nsTableRowGroupFrame::NoRowsFollow()
{
  PRBool result = PR_TRUE;
  nsIFrame *nextSib=nsnull;
  GetNextSibling(nextSib);
  while (nsnull!=nextSib)
  {
    const nsStyleDisplay *sibDisplay;
    nextSib->GetStyleData(eStyleStruct_Display, ((nsStyleStruct *&)sibDisplay));
    if ((NS_STYLE_DISPLAY_TABLE_HEADER_GROUP == sibDisplay->mDisplay) ||
        (NS_STYLE_DISPLAY_TABLE_FOOTER_GROUP == sibDisplay->mDisplay) ||
        (NS_STYLE_DISPLAY_TABLE_ROW_GROUP    == sibDisplay->mDisplay))
    {
      nsIFrame *childFrame=nsnull;
      nextSib->FirstChild(childFrame);
      while (nsnull!=childFrame)
      {
        const nsStyleDisplay *childDisplay;
        childFrame->GetStyleData(eStyleStruct_Display, ((nsStyleStruct *&)childDisplay));
        if (NS_STYLE_DISPLAY_TABLE_ROW == childDisplay->mDisplay)
        { // found a row 
          result = PR_FALSE;
          break;
        }
      }
    }
    nextSib->GetNextSibling(nextSib);
  }
  if (PR_TRUE==gsDebugIR) printf("\nTRGF IR: NoRowsFollow returning %d\n", result);
  return result;
}

// since we know we're doing an append here, we can optimize
NS_METHOD nsTableRowGroupFrame::IR_RowAppended(nsIPresContext&      aPresContext,
                                               nsHTMLReflowMetrics& aDesiredSize,
                                               RowGroupReflowState& aReflowState,
                                               nsReflowStatus&      aStatus,
                                               nsTableRowFrame *    aAppendedFrame)
{
  if (PR_TRUE==gsDebugIR) printf("\nTRGF IR: IR_RowAppended\n");
  // hook aAppendedFrame into the child list
  nsresult rv = AddFrame(aReflowState.reflowState, (nsIFrame*)aAppendedFrame);
  if (NS_FAILED(rv))
    return rv;

  /* we have 2 paths to choose from.  If we know that aAppendedFrame is
   * the last row in the table, we can optimize.  Otherwise, we have to
   * treat it like an insert
   */
  if (PR_TRUE==NoRowsFollow())
  { // aAppendedRow is the last row, so do the optimized route
    // account for the cells in the row aAppendedFrame
    // this will add the content of the rowgroup to the cell map
    rv = DidAppendRow(aAppendedFrame);
    if (NS_FAILED(rv))
      return rv;

    nsTableFrame *tableFrame=nsnull;
    rv = nsTableFrame::GetTableFrame(this, tableFrame);
    if (NS_FAILED(rv) || nsnull==tableFrame)
      return rv;

    if (PR_TRUE==tableFrame->RequiresPass1Layout())
    {
      // do a pass1 reflow of the new row
      //XXX: check the table frame to see if we can skip this
      rv = ReflowMappedChildren(aPresContext, aDesiredSize, aReflowState, aStatus, 
                                aAppendedFrame, eReflowReason_Initial, PR_FALSE);
      if (NS_FAILED(rv))
        return rv;
    }

    // if any column widths have to change due to this, rebalance column widths
    //XXX need to calculate this, but for now just do it
    tableFrame->InvalidateColumnWidths();  
  }
  else
  {
    // do a pass1 reflow of the new row
    //XXX: check the table frame to see if we can skip this
    rv = ReflowMappedChildren(aPresContext, aDesiredSize, aReflowState, aStatus, 
                              aAppendedFrame, eReflowReason_Initial, PR_FALSE);
    if (NS_FAILED(rv))
      return rv;

    nsTableFrame *tableFrame=nsnull;
    rv = nsTableFrame::GetTableFrame(this, tableFrame);
    if (NS_FAILED(rv) || nsnull==tableFrame)
      return rv;
    tableFrame->InvalidateCellMap();
    tableFrame->InvalidateColumnCache();
  }

  return rv;
}

NS_METHOD nsTableRowGroupFrame::IR_RowRemoved(nsIPresContext&      aPresContext,
                                              nsHTMLReflowMetrics& aDesiredSize,
                                              RowGroupReflowState& aReflowState,
                                              nsReflowStatus&      aStatus,
                                              nsTableRowFrame *    aDeletedFrame)
{
  if (PR_TRUE==gsDebugIR) printf("\nTRGF IR: IR_RowRemoved\n");
  nsresult rv = RemoveFrame((nsIFrame *)aDeletedFrame);
  if (NS_SUCCEEDED(rv))
  {
    nsTableFrame *tableFrame=nsnull;
    rv = nsTableFrame::GetTableFrame(this, tableFrame);
    if (NS_FAILED(rv) || nsnull==tableFrame)
      return rv;
    tableFrame->InvalidateCellMap();
    tableFrame->InvalidateColumnCache();

    // if any column widths have to change due to this, rebalance column widths
    //XXX need to calculate this, but for now just do it
    tableFrame->InvalidateColumnWidths();
  }

  return rv;
}

NS_METHOD nsTableRowGroupFrame::IR_TargetIsChild(nsIPresContext&      aPresContext,
                                                 nsHTMLReflowMetrics& aDesiredSize,
                                                 RowGroupReflowState& aReflowState,
                                                 nsReflowStatus&      aStatus,
                                                 nsIFrame *           aNextFrame)

{
  nsresult rv;
  if (PR_TRUE==gsDebugIR) printf("\nTRGF IR: IR_TargetIsChild\n");
  // XXX Recover state

  // Remember the old rect
  nsRect  oldKidRect;
  aNextFrame->GetRect(oldKidRect);

  // Pass along the reflow command
  // XXX Correctly compute the available space...
  nsHTMLReflowState   kidReflowState(aPresContext, aNextFrame, aReflowState.reflowState, aReflowState.reflowState.maxSize);
  nsHTMLReflowMetrics desiredSize(nsnull);

  rv = ReflowChild(aNextFrame, aPresContext, desiredSize, kidReflowState, aStatus);

  // Resize the row frame
  nsRect  kidRect;
  aNextFrame->GetRect(kidRect);
  aNextFrame->SizeTo(desiredSize.width, desiredSize.height);

  // Adjust the frames that follow...
  AdjustSiblingsAfterReflow(aPresContext, aReflowState, aNextFrame, desiredSize.height -
                            oldKidRect.height);

  // Return of desired size
  aDesiredSize.width = aReflowState.reflowState.maxSize.width;
  aDesiredSize.height = aReflowState.y;

  return rv;
}

NS_METHOD nsTableRowGroupFrame::IR_StyleChanged(nsIPresContext&      aPresContext,
                                                nsHTMLReflowMetrics& aDesiredSize,
                                                RowGroupReflowState& aReflowState,
                                                nsReflowStatus&      aStatus)
{
  if (PR_TRUE==gsDebugIR) printf("TRGF: IR_StyleChanged for frame %p\n", this);
  nsresult rv = NS_OK;
  // we presume that all the easy optimizations were done in the nsHTMLStyleSheet before we were called here
  // XXX: we can optimize this when we know which style attribute changed
  nsTableFrame* tableFrame=nsnull;
  rv = nsTableFrame::GetTableFrame(this, tableFrame);
  if ((NS_SUCCEEDED(rv)) && (nsnull!=tableFrame))
  {
    tableFrame->InvalidateFirstPassCache();
  }
  return rv;
}

NS_METHOD
nsTableRowGroupFrame::CreateContinuingFrame(nsIPresContext&  aPresContext,
                                            nsIFrame*        aParent,
                                            nsIStyleContext* aStyleContext,
                                            nsIFrame*&       aContinuingFrame)
{
  nsTableRowGroupFrame* cf = new nsTableRowGroupFrame(mContent, aParent);
  if (nsnull == cf) {
    return NS_ERROR_OUT_OF_MEMORY;
  }
  PrepareContinuingFrame(aPresContext, aParent, aStyleContext, cf);
  if (PR_TRUE==gsDebug) printf("nsTableRowGroupFrame::CCF parent = %p, this=%p, cf=%p\n", aParent, this, cf);
  aContinuingFrame = cf;
  return NS_OK;
}

/* ----- global methods ----- */

nsresult 
NS_NewTableRowGroupFrame(nsIContent* aContent,
                         nsIFrame*   aParentFrame,
                         nsIFrame*&  aResult)
{
  nsIFrame* it = new nsTableRowGroupFrame(aContent, aParentFrame);
  if (nsnull == it) {
    return NS_ERROR_OUT_OF_MEMORY;
  }
  aResult = it;
  return NS_OK;
}

/* ----- debugging methods ----- */
NS_METHOD nsTableRowGroupFrame::List(FILE* out, PRInt32 aIndent, nsIListFilter *aFilter) const
{
  // if a filter is present, only output this frame if the filter says we should
  // since this could be any "tag" with the right display type, we'll
  // just pretend it's a tbody
  if (nsnull==aFilter)
    return nsContainerFrame::List(out, aIndent, aFilter);

  nsAutoString tagString("tbody");
  PRBool outputMe = aFilter->OutputTag(&tagString);
  if (PR_TRUE==outputMe)
  {
    // Indent
    for (PRInt32 i = aIndent; --i >= 0; ) fputs("  ", out);

    // Output the tag and rect
    nsIAtom* tag;
    mContent->GetTag(tag);
    if (tag != nsnull) {
      nsAutoString buf;
      tag->ToString(buf);
      fputs(buf, out);
      NS_RELEASE(tag);
    }

    fprintf(out, "(%d)", ContentIndexInContainer(this));
    out << mRect;
    if (0 != mState) {
      fprintf(out, " [state=%08x]", mState);
    }
    fputs("\n", out);
  }

  // Output the children
  if (nsnull != mFirstChild) {
    if (PR_TRUE==outputMe)
    {
      if (0 != mState) {
        fprintf(out, " [state=%08x]\n", mState);
      }
    }
    for (nsIFrame* child = mFirstChild; child; child->GetNextSibling(child)) {
      child->List(out, aIndent + 1, aFilter);
    }
  } else {
    if (PR_TRUE==outputMe)
    {
      if (0 != mState) {
        fprintf(out, " [state=%08x]\n", mState);
      }
    }
  }


  return NS_OK;
}






